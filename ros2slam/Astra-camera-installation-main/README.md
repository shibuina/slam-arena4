# Astra-camera-installation

Download your sdk from here.
```
https://www.orbbec.com/developers/openni-sdk/
```

Preinstall 

```
sudo apt-get install build-essential freeglut3 freeglut3-dev
```

Extract sdk zip file, chose your sdk for your os, extract one more, open rules file.

```
bash bash install.sh
```

After installation, go to 

```
<Your sdk version>/sdk
```

install openni
```
pip install openni
sudo pip3 install openni #if you use tx2
```

Copy and replace libs to folder contain test.py
replug your camera and have fun.
