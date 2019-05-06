all: buggy fixed

buggy:
	docker build -t roswire/helloworld:buggy .

fixed:
	docker build --target base -t roswire/helloworld:fixed .

.PHONY: buggy fixed
