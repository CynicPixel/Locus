// main2.go
// This file demonstrates how to parse the structured byte stream from sellers
// and extract raw ModeS messages, timestamps, and sensor positions.
//
// INSTRUCTIONS: To use this file, you must comment out or rename main.go first.
// Go does not allow two main() functions in the same package.
//
// After commenting out main.go, run:
//
//	go run main.go --port=6653 --mode=peer --buyer-or-seller=buyer --list-of-sellers-source=env --envFile=.buyer-env
package main

import (
	"context"
	"encoding/binary"
	"fmt"
	"io"
	"log"
	"math"
	"time"

	neuronsdk "github.com/NeuronInnovations/neuron-go-hedera-sdk" // Import neuronFactory from neuron-go-sdk
	commonlib "github.com/NeuronInnovations/neuron-go-hedera-sdk/common-lib"

	"github.com/hashgraph/hedera-sdk-go/v2"
	"github.com/libp2p/go-libp2p/core/host"
	"github.com/libp2p/go-libp2p/core/network"
	"github.com/libp2p/go-libp2p/core/protocol"
)

const (
	sensorIDSize             = 8 // int64 is 8 bytes
	sensorLongitudeSize      = 8 // float64 is 8 bytes
	sensorLatitudeSize       = 8 // float64 is 8 bytes
	sensorAltitudeSize       = 8 // float64 is 8 bytes
	secondsSinceMidnightSize = 8 // uint64 is 8 bytes
	nanosecondsSize          = 8 // uint64 is 8 bytes
	minFixedSize             = sensorIDSize + sensorLatitudeSize + sensorLongitudeSize + sensorAltitudeSize + secondsSinceMidnightSize + nanosecondsSize
)

// A helper to read exactly len(buf) bytes from the stream
func readExact(s network.Stream, buf []byte) error {
	var total int
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

func float64FromByte(bytes []byte) float64 {
	bits := binary.BigEndian.Uint64(bytes)
	return math.Float64frombits(bits)
}

func int64FromByte(bytes []byte) int64 {
	bits := binary.BigEndian.Uint64(bytes)
	return int64(bits)
}

func uint64FromByte(bytes []byte) uint64 {
	return binary.BigEndian.Uint64(bytes)
}

func main() {
	//"neuron/ADSB/0.0.2"
	var NrnProtocol = protocol.ID("neuron/ADSB/0.0.2")

	neuronsdk.LaunchSDK(
		"0.1",       // Specify your app's version
		NrnProtocol, // Specify a protocol ID
		nil,         // leave nil if you don't need custom key configuration logic
		func(ctx context.Context, h host.Host, b *commonlib.NodeBuffers) { // Define buyer case logic here (if required)
			h.SetStreamHandler(NrnProtocol, func(streamHandler network.Stream) {
				defer streamHandler.Close()
				// I am receiving data from the following peer
				peerID := streamHandler.Conn().RemotePeer()
				b.SetStreamHandler(peerID, &streamHandler)

				fmt.Printf("Stream established with peer %s\n", peerID)

				for {
					isStreamClosed := network.Stream.Conn(streamHandler).IsClosed()
					if isStreamClosed {
						log.Println("Stream seems to be closed ...", peerID)
						break
					}

					// Set a read deadline to avoid blocking indefinitely
					streamHandler.SetReadDeadline(time.Now().Add(5 * time.Second))

					// 1) Read the first byte that tells total message length
					lengthBuf := make([]byte, 1)
					if err := readExact(streamHandler, lengthBuf); err != nil {
						if err != io.EOF {
							log.Printf("Error reading length byte: %v", err)
						}
						break
					}

					totalPacketSize := int(lengthBuf[0])
					if totalPacketSize == 0 {
						log.Println("Got a 0-length payload; ignoring")
						continue
					}

					// 2) Read the rest of the packet
					packetBytes := make([]byte, totalPacketSize)
					if err := readExact(streamHandler, packetBytes); err != nil {
						if err != io.EOF {
							log.Printf("Error reading packet: %v", err)
						}
						break
					}

					// Check if packet is large enough
					if len(packetBytes) < minFixedSize {
						fmt.Println("Packet too short, ignoring")
						continue
					}

					offset := 0

					// (1) sensorID (8 bytes => int64)
					sensorID := int64FromByte(packetBytes[offset : offset+sensorIDSize])
					offset += sensorIDSize

					// (2) latitude (8 bytes => float64)
					sensorLatitude := float64FromByte(packetBytes[offset : offset+sensorLatitudeSize])
					offset += sensorLatitudeSize

					// (3) longitude (8 bytes => float64)
					sensorLongitude := float64FromByte(packetBytes[offset : offset+sensorLongitudeSize])
					offset += sensorLongitudeSize

					// (4) altitude (8 bytes => float64)
					sensorAltitude := float64FromByte(packetBytes[offset : offset+sensorAltitudeSize])
					offset += sensorAltitudeSize

					// (5) secondsSinceMidnight (8 bytes => uint64)
					secondsSinceMidnight := uint64FromByte(packetBytes[offset : offset+secondsSinceMidnightSize])
					offset += secondsSinceMidnightSize

					// (6) nanoseconds (8 bytes => uint64)
					nanoseconds := uint64FromByte(packetBytes[offset : offset+nanosecondsSize])
					offset += nanosecondsSize

					// (7) rawModeS (remaining bytes)
					rawModeS := packetBytes[offset:]

					// Print the parsed information
					fmt.Printf("=== ModeS Message from Peer %s ===\n", peerID)
					fmt.Printf("Sensor ID: %d\n", sensorID)
					fmt.Printf("Sensor Position: Lat=%.6f, Lon=%.6f, Alt=%.2f\n", sensorLatitude, sensorLongitude, sensorAltitude)
					fmt.Printf("Timestamp: SecondsSinceMidnight=%d, Nanoseconds=%d\n", secondsSinceMidnight, nanoseconds)
					fmt.Printf("Raw ModeS (hex): %x\n", rawModeS)
					fmt.Printf("Raw ModeS (bytes): %v\n", rawModeS)
					fmt.Printf("Raw ModeS length: %d bytes\n", len(rawModeS))
					fmt.Println("---")
				}
			})

		},
		func(msg hedera.TopicMessage) { // Define buyer topic callback logic here (if required)
			fmt.Println(msg)
		},
		func(ctx context.Context, h host.Host, b *commonlib.NodeBuffers) { // Define seller case logic here (if required)
			// every 10 seconds, send a ping message

		},
		func(msg hedera.TopicMessage) {
			// Define seller topic callback logic here (if required)
		},
	)
}
