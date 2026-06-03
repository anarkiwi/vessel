# Host-based test environment for Vessel.
#
# Builds and runs the native unit tests (GoogleTest) for the firmware's parsing
# logic, plus the clang-format check. No embedded toolchain or hardware needed.
#
#   docker build -t vessel-test .      # builds image AND runs the tests
#   docker run --rm vessel-test        # re-run the tests
#
# Ubuntu 24.04 ships clang-format 18, matching the version used to format the
# tree, so the format check is deterministic.
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    clang-format \
    cmake \
    git \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /vessel
COPY . .

# Configure + build (fetches pinned GoogleTest and the real MIDI library), then
# run the full test suite. The build fails if either the tests or the format
# check fail.
RUN cmake -S test/host -B build \
  && cmake --build build -j"$(nproc)" \
  && ctest --test-dir build --output-on-failure

CMD ["ctest", "--test-dir", "build", "--output-on-failure"]
