FROM rust:1.79-slim-bookworm AS builder
WORKDIR /src
COPY Cargo.toml Cargo.lock ./
# cache dependencies layer
RUN mkdir src && echo "fn main(){}" > src/main.rs && cargo build --release && rm -rf src
COPY src ./src
RUN touch src/main.rs && cargo build --release

FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates && rm -rf /var/lib/apt/lists/*
COPY --from=builder /src/target/release/locus-backend /usr/local/bin/locus-backend
VOLUME ["/tmp/locus"]
EXPOSE 9002
ENTRYPOINT ["locus-backend"]
