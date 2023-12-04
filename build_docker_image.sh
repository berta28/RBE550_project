#! /bin/bash
VERSION=$(cat version.txt)
docker build \
	--no-cache \
	-t "rbe550_project:${VERSION}" \
	-f Dockerfile . 
