FROM golang:1.23-bookworm AS builder
WORKDIR /src
COPY go.mod go.sum ./
RUN go mod download
COPY . .
RUN CGO_ENABLED=0 GOOS=linux go build -o /ingestor .

FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates && rm -rf /var/lib/apt/lists/*
COPY --from=builder /ingestor /usr/local/bin/ingestor
COPY .buyer-env.example /etc/locus/buyer-env.example
VOLUME ["/tmp/locus"]
ENTRYPOINT ["ingestor"]
