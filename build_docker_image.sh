#! /bin/bash
VERSION=$(cat version.txt)
docker build \
	-t "rbe550_project:${VERSION}" \
	-f Dockerfile . 