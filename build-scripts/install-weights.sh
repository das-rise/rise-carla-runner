ZIPNAME="pretrained.zip"

if [ ! -f pretrained.zip ]; then
    echo "Downloading pretrained weights, this may take a while..."
    wget https://zenodo.org/records/17110968/files/pretrained.zip?download=1 -O "$ZIPNAME"
fi
unzip "$ZIPNAME" -d ../src/rirun/PCLA/agents/.