#! /bin/bash
docker build \
	--progress plain \
	-t "rbe550_project:0.0.0" \
	-f Dockerfile . 