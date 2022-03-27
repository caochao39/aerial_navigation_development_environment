#!/bin/bash

cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

echo ""
echo "Downloading simulation environments..."
echo ""

ggID='1W_KicQUrEEv15_zqN-ZQpjdSuxew-u6p'
ggURL='https://drive.google.com/uc?export=download'

filename="$(curl -sc /tmp/gcokie "${ggURL}&id=${ggID}" | grep -o '="uc-name.*</span>' | sed 's/.*">//;s/<.a> .*//')"
html=`curl -c /tmp/gcokie -s -L "https://drive.google.com/uc?export=download&id=${ggID}"`
curl -Lb /tmp/gcokie "https://drive.google.com/uc?export=download&`echo ${html}|grep -Po '(confirm=[a-zA-Z0-9\-_]+)'`&id=${ggID}" -o ${filename}

echo ""
echo "Unzipping files..."
echo ""

unzip "${filename}"

rm "${filename}"

echo ""
echo "Done, simulation environments are kept in 'src/vehicle_simulator/mesh'."
echo ""
