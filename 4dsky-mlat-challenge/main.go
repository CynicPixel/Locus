// main.go
package main

import (
	"bufio"
	"context"
	"fmt"
	"log"
	"time"

	neuronsdk "github.com/NeuronInnovations/neuron-go-hedera-sdk" // Import neuronFactory from neuron-go-sdk
	commonlib "github.com/NeuronInnovations/neuron-go-hedera-sdk/common-lib"

	"github.com/hashgraph/hedera-sdk-go/v2"
	"github.com/libp2p/go-libp2p/core/host"
	"github.com/libp2p/go-libp2p/core/network"
	"github.com/libp2p/go-libp2p/core/protocol"
)

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
				streamReader := bufio.NewReader(streamHandler)

				fmt.Printf("Stream established with peer %s\n", peerID)

				// print ping messages to the screen while the other side sends data
				for {
					isStreamClosed := network.Stream.Conn(streamHandler).IsClosed()
					if isStreamClosed {
						log.Println("Stream seems to be closed ...", peerID)
						break
					}

					// Set a read deadline to avoid blocking indefinitely
					streamHandler.SetReadDeadline(time.Now().Add(5 * time.Second))

					// Try to read a byte
					bytesFromOtherside, err := streamReader.ReadByte()

					if err != nil {
						continue
					}

					// If we got here, we successfully read data with a newline
					fmt.Printf("Received from %s: %x\n", peerID, bytesFromOtherside)
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
