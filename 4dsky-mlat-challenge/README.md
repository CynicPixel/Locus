# 4dsky MLAT Challenge

A quickstart project to help you understand how to buy 4dskyEdge data and perform multilateration using the Neuron Go Hedera SDK.

## Overview

This program is here to help you understand buying 4dskyEdge data. The application demonstrates how to connect to sellers and receive ModeS data streams over the Hedera network using the Neuron SDK. The buyer implementation is functional and will receive raw ModeS data from any connected seller peer. The seller case is currently an empty and doesn't need to be implemented.

## The 4dsky MLAT Challenge

This repository is set up as a challenge to help you understand multilateration (MLAT) using ModeS data.

### Challenge Steps:
0. Get credentials from neuron (see below how), port forward 61336 on your router. 
1. **Try the simple main file** - Get some bytes
   - Start with `main.go` to understand the basic connection
   - Run it and observe the raw byte stream coming from sellers
   - This helps you see how the connection works

2. **Try the second main file** - Get raw data
   - Rename `main.go` to `main.go.bak`
   - Rename `main-half-4dsky.go.bak` to `main.go`
   - Run it again and observe the structured data output
   - You'll see sensor positions, timestamps, and raw ModeS frames



3. **The Challenge: Can you multilaterate the frames?**
   - The timestamps are super accurate (nanosecond precision)
   - You have sensor positions (latitude, longitude, altitude) for each frame
   - You have raw ModeS data with precise timestamps
   - **Can you use multilateration to calculate aircraft positions?**

### What is Multilateration?

Multilateration (MLAT) is a technique that uses the time difference of arrival (TDOA) of signals from multiple sensors to determine the position of an aircraft. With accurate timestamps and known sensor positions, you can calculate where an aircraft is located.

## Get Your Credentials

**You will need buyer credentials and a list of sellers to buy from.**

**Some sellers don't reveal their true location, you will also receive the exact locations to override their reported location**

To kick things off: join our Discord channel to get started: 
https://discord.gg/PeAbrrrq7Z

Simply come join, say hi and say that you need creds and locations for the neuron 4Dsky challenge. We'll help you get set up with:
- Buyer credentials to connect to the network
- A list of sellers and locations to buy from (available only on Hackathon kickoff day)
- Any help you need along the way

If you need help or want to discuss solutions during the challenge, the Discord is the place to be!

## Prerequisites

### Environment
- **VS Code** (Visual Studio Code) - Recommended IDE for running this project
- **Go Extension for VS Code** - Required for Go language support and debugging
- **Port forward 61336 on your router**

### Dependencies
- **Golang** (Go) - Version 1.24.6 or compatible (see `go.mod` for exact version)
- Go toolchain 1.24.10

### Network Configuration
**IMPORTANT:** In order to run these files, you need to port forward at your router (open the port) that is declared in the launcher configuration. The buyer mode uses port `61336` in this configuration.

## Setup

1. **Clone or download this repository**

2. **Install dependencies:**
   ```bash
   go mod download
   ```

3. **Set up environment files:**
   - Copy `.buyer-env.example` to `.buyer-env` and fill in your actual credentials from Discord
   - Make sure to include the `list_of_sellers` field with the seller peer IDs you received
   
   **Note:** The `.env` files contain sensitive information and are gitignored. Never commit them to version control.

## Running the Application

### Running in Buyer Mode

**To run this demo, you need the buyer environment file (`.buyer-env`) and launch in buyer mode.**

The easiest way to run the application in buyer mode is using the VS Code launch configuration:

1. Open the project in **VS Code**
2. Ensure you have the **Go extension** installed
3. Navigate to the Run and Debug panel (or press `F5`)
4. Select **"Run buyer"** from the configuration dropdown
5. Click the play button or press `F5` to start

The launch configuration will automatically:
- Run the program in debug mode
- Use port `61336`
- Set mode to `peer`
- Configure the buyer role
- Load environment variables from `.buyer-env`

### Manual Command Line Execution

Alternatively, you can run the buyer mode from the command line:

```bash
go run main.go --port=61336 --mode=peer --buyer-or-seller=buyer --list-of-sellers-source=env --envFile=.buyer-env
```

## How It Works

### Buyer Mode

When running in buyer mode, the application will:

1. Connect to sellers specified in your `.buyer-env` file
2. Establish peer-to-peer streams using the `neuron/ADSB/0.0.2` protocol
3. Receive **super raw ModeS streams** from connected seller peers
4. Print received data to the console

The buyer implementation includes:
- Stream handler setup for receiving data
- Connection management and monitoring
- Topic message callback for Hedera topic messages

### Seller Mode

The seller case is currently an **empty stub** and you don't need to fill it out for this challenge.

## Two Main Files - Instructional Examples

This repository contains two main files for instructional purposes:

### 1. `main.go` - Simple Connection Example

The `main.go` file is a very simple example where only the buyer block is declared and it reads one byte at a time from whoever you are connected to. That's pretty useless for actual data processing, but it helps you see how to connect and establish a stream with a seller.

**What it does:**
- Establishes a connection with sellers
- Reads one byte at a time from the stream
- Prints each byte in hexadecimal format
- Demonstrates basic stream handling

**To run:**
```bash
go run main.go --port=61336 --mode=peer --buyer-or-seller=buyer --list-of-sellers-source=env --envFile=.buyer-env
```

### 2. `main-half-4dsky.go.bak` - Structured Data Parsing Example

The `main-half-4dsky.go.bak` file demonstrates how to parse the structured byte stream format. This time you will see that you are unpacking the structure of what is read and you are receiving ModeS data raw.

**What it does:**
- Reads the length-prefixed packet structure
- Extracts sensor ID, position (lat/lon/alt), timestamps, and raw ModeS data
- Prints all parsed information to the console
- Does NOT convert to positional messages - only shows raw ModeS and metadata

**To use this file:**
1. Rename `main.go` to `main.go.bak` (or any backup name)
2. Rename `main-half-4dsky.go.bak` to `main.go`
3. Run it again using the same command:
   ```bash
   go run main.go --port=61336 --mode=peer --buyer-or-seller=buyer --list-of-sellers-source=env --envFile=.buyer-env
   ```

This time you will see structured output showing:
- Sensor ID (int64)
- Sensor position (latitude, longitude, altitude)
- Timestamps (seconds since midnight, nanoseconds)
- Raw ModeS message data (hex and byte format)

## Protocol

This application uses the `neuron/ADSB/0.0.2` protocol for peer-to-peer communication over the Hedera network.

## Configuration

### Buyer Environment Variables (`.buyer-env`)
- `eth_rpc_url` - Ethereum RPC endpoint
- `hedera_evm_id` - Hedera EVM account ID
- `hedera_id` - Hedera account ID
- `location` - Geographic location (lat/lon/alt)
- `mirror_api_url` - Hedera mirror node API URL
- `private_key` - Private key for authentication
- `smart_contract_address` - Smart contract address
- `list_of_sellers` - List of seller peer IDs to connect to


## Development

The project structure:
- `main.go` - Simple example that reads one byte at a time (connection demonstration)
- `main-half-4dsky.go.bak` - Structured parsing example (rename to `main.go` to use)
- `.vscode/launch.json` - VS Code debug configurations
- `.buyer-env.example` / `.seller-env.example` - Environment file templates
- `go.mod` / `go.sum` - Go module dependencies

## Notes

- **main.go**: Receives raw ModeS data as individual bytes and prints them in hexadecimal format (`%x`). Useful for understanding the connection process.
- **main-half-4dsky.go.bak**: Parses the structured byte stream format and prints:
  - Sensor ID (int64)
  - Sensor position (latitude, longitude, altitude)
  - Timestamps (seconds since midnight, nanoseconds)
  - Raw ModeS message data (hex and byte format)
- Stream connections are monitored and will log when closed
- The seller implementation is a placeholder
- Both main files demonstrate different approaches to reading the seller's data stream
- **Remember to port forward the ports in your router** (buyer: 61336)

## License

See the repository license file for details.
