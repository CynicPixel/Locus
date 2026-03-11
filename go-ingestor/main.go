package main

import (
	"context"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"math"
	"net"
	"os"
	"sync"
	"time"

	neuronsdk "github.com/NeuronInnovations/neuron-go-hedera-sdk"
	commonlib "github.com/NeuronInnovations/neuron-go-hedera-sdk/common-lib"
	"github.com/hashgraph/hedera-sdk-go/v2"
	"github.com/libp2p/go-libp2p/core/host"
	"github.com/libp2p/go-libp2p/core/network"
	"github.com/libp2p/go-libp2p/core/peer"
	"github.com/libp2p/go-libp2p/core/protocol"
)

// ---------------------------------------------------------------------------
// Wire format constants (from 4DSky SDK binary protocol)
// ---------------------------------------------------------------------------

const (
	sensorIDSize             = 8
	sensorLatitudeSize       = 8
	sensorLongitudeSize      = 8
	sensorAltitudeSize       = 8
	secondsSinceMidnightSize = 8
	nanosecondsSize          = 8
	minFixedSize             = sensorIDSize + sensorLatitudeSize + sensorLongitudeSize +
		sensorAltitudeSize + secondsSinceMidnightSize + nanosecondsSize
)

// ---------------------------------------------------------------------------
// Output JSON type
// ---------------------------------------------------------------------------

// RawFrameOut is the JSON line emitted per received Mode-S message.
type RawFrameOut struct {
	SensorID             int64   `json:"sensor_id"`
	SensorLat            float64 `json:"sensor_lat"`
	SensorLon            float64 `json:"sensor_lon"`
	SensorAlt            float64 `json:"sensor_alt"`
	TimestampSeconds     uint64  `json:"timestamp_seconds"`
	TimestampNanoseconds uint64  `json:"timestamp_nanoseconds"`
	RawModesHex          string  `json:"raw_modes_hex"`
}

// ---------------------------------------------------------------------------
// Location override (seller GPS correction map)
// ---------------------------------------------------------------------------

type locationEntry struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
	Alt float64 `json:"alt"`
}

func loadLocationOverrides(path string) map[string]locationEntry {
	f, err := os.Open(path)
	if err != nil {
		log.Printf("location-override: could not open %s: %v", path, err)
		return nil
	}
	defer f.Close()

	var m map[string]locationEntry
	if err := json.NewDecoder(f).Decode(&m); err != nil {
		log.Printf("location-override: parse error: %v", err)
		return nil
	}
	log.Printf("location-override: loaded %d entries", len(m))
	return m
}

// ---------------------------------------------------------------------------
// Binary helpers
// ---------------------------------------------------------------------------

func readExact(s network.Stream, buf []byte) error {
	total := 0
	for total < len(buf) {
		n, err := s.Read(buf[total:])
		if err != nil {
			return err
		}
		if n == 0 {
			return io.EOF
		}
		total += n
	}
	return nil
}

func float64FromBytes(b []byte) float64 {
	return math.Float64frombits(binary.BigEndian.Uint64(b))
}

func int64FromBytes(b []byte) int64  { return int64(binary.BigEndian.Uint64(b)) }
func uint64FromBytes(b []byte) uint64 { return binary.BigEndian.Uint64(b) }

// ---------------------------------------------------------------------------
// Peer public key extraction for location override lookup
// ---------------------------------------------------------------------------

func peerIDToCompressedPubKeyHex(pid peer.ID) (string, error) {
	pub, err := pid.ExtractPublicKey()
	if err != nil {
		return "", fmt.Errorf("peer.ID without embedded public key: %w", err)
	}
	raw, err := pub.Raw()
	if err != nil {
		return "", err
	}
	return hex.EncodeToString(raw), nil
}

// ---------------------------------------------------------------------------
// Unix socket broadcast machinery
// ---------------------------------------------------------------------------

const socketPath = "/tmp/locus/ingestor.sock"

var (
	clientsMu sync.Mutex
	clients   = make(map[net.Conn]struct{})
)

func startSocketListener() {
	if err := os.MkdirAll("/tmp/locus", 0o755); err != nil {
		log.Fatalf("mkdir /tmp/locus: %v", err)
	}
	if err := os.Remove(socketPath); err != nil && !os.IsNotExist(err) {
		log.Fatalf("remove stale socket: %v", err)
	}

	ln, err := net.Listen("unix", socketPath)
	if err != nil {
		log.Fatalf("bind unix socket: %v", err)
	}
	log.Printf("Ingestor listening on %s", socketPath)

	go func() {
		for {
			conn, err := ln.Accept()
			if err != nil {
				log.Printf("accept error: %v", err)
				continue
			}
			clientsMu.Lock()
			clients[conn] = struct{}{}
			clientsMu.Unlock()
			log.Printf("Rust backend connected (%s)", conn.RemoteAddr())
		}
	}()
}

func broadcast(data []byte) {
	clientsMu.Lock()
	defer clientsMu.Unlock()
	for conn := range clients {
		if err := conn.SetWriteDeadline(time.Now().Add(100 * time.Millisecond)); err != nil {
			conn.Close()
			delete(clients, conn)
			continue
		}
		if _, err := fmt.Fprintf(conn, "%s\n", data); err != nil {
			log.Printf("client write error, removing: %v", err)
			conn.Close()
			delete(clients, conn)
		}
	}
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

func main() {
	overrides := loadLocationOverrides("./location-override.json")

	startSocketListener()

	var NrnProtocol = protocol.ID("neuron/ADSB/0.0.2")

	neuronsdk.LaunchSDK(
		"0.1",
		NrnProtocol,
		nil,
		func(ctx context.Context, h host.Host, b *commonlib.NodeBuffers) {
			h.SetStreamHandler(NrnProtocol, func(streamHandler network.Stream) {
				defer streamHandler.Close()

				peerID := streamHandler.Conn().RemotePeer()
				b.SetStreamHandler(peerID, &streamHandler)

				pubKeyHex, err := peerIDToCompressedPubKeyHex(peerID)
				if err != nil {
					log.Printf("could not extract pubkey for %s: %v", peerID, err)
				}

				log.Printf("Stream established with peer %s", peerID)

				for {
					if network.Stream.Conn(streamHandler).IsClosed() {
						log.Printf("Stream closed: %s", peerID)
						break
					}

					streamHandler.SetReadDeadline(time.Now().Add(5 * time.Second))

					// 1) Length byte
					var lenBuf [1]byte
					if err := readExact(streamHandler, lenBuf[:]); err != nil {
						if err != io.EOF {
							log.Printf("read length: %v", err)
						}
						break
					}

					totalPacketSize := int(lenBuf[0])
					if totalPacketSize == 0 {
						continue
					}

					// 2) Packet body
					pkt := make([]byte, totalPacketSize)
					if err := readExact(streamHandler, pkt); err != nil {
						if err != io.EOF {
							log.Printf("read packet: %v", err)
						}
						break
					}

					if len(pkt) < minFixedSize {
						log.Printf("packet too short (%d bytes)", len(pkt))
						continue
					}

					off := 0
					sensorID := int64FromBytes(pkt[off : off+sensorIDSize])
					off += sensorIDSize
					sensorLat := float64FromBytes(pkt[off : off+sensorLatitudeSize])
					off += sensorLatitudeSize
					sensorLon := float64FromBytes(pkt[off : off+sensorLongitudeSize])
					off += sensorLongitudeSize
					sensorAlt := float64FromBytes(pkt[off : off+sensorAltitudeSize])
					off += sensorAltitudeSize
					secondsSinceMidnight := uint64FromBytes(pkt[off : off+secondsSinceMidnightSize])
					off += secondsSinceMidnightSize
					nanoseconds := uint64FromBytes(pkt[off : off+nanosecondsSize])
					off += nanosecondsSize
					rawModeS := pkt[off:]

					// Apply location override if available
					if pubKeyHex != "" {
						if ov, ok := overrides[pubKeyHex]; ok {
							sensorLat = ov.Lat
							sensorLon = ov.Lon
							sensorAlt = ov.Alt
						}
					}

					// Convert seconds-since-midnight to Unix epoch
					now := time.Now().UTC()
					midnight := time.Date(now.Year(), now.Month(), now.Day(), 0, 0, 0, 0, time.UTC)

					frame := RawFrameOut{
						SensorID:             sensorID,
						SensorLat:            sensorLat,
						SensorLon:            sensorLon,
						SensorAlt:            sensorAlt,
						TimestampSeconds:     uint64(midnight.Unix()) + secondsSinceMidnight,
						TimestampNanoseconds: nanoseconds,
						RawModesHex:          fmt.Sprintf("%x", rawModeS),
					}

					jsonBytes, err := json.Marshal(frame)
					if err != nil {
						log.Printf("json marshal: %v", err)
						continue
					}
					broadcast(jsonBytes)
				}
			})
		},
		func(msg hedera.TopicMessage) {},
		func(ctx context.Context, h host.Host, b *commonlib.NodeBuffers) {},
		func(msg hedera.TopicMessage) {},
	)
}
