.PHONY: build run test clean

build:
	go build -o bin/app

run:
	go run main.go

test:
	go test ./...

clean:
	rm -rf bin/
