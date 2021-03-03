# deepracer-sim

DeepRacer Simulator

## init

```bash
# on Mac or Ubuntu
brew install pyenv
pyenv install 3.8.6

pyenv shell 3.8.6

pip3 install numpy
pip3 install pygame

# on Windows
py -m pip install -U pygame --user
```

## run

```bash
python3 sim.py
python3 sim.py -ad -s 1.5
python3 sim.py -ad -s 1.5 --bots-count 6 --bots-speed 1.0
```

[![DeepRacer Simulator](http://img.youtube.com/vi/9jSZm7FcqmE/0.jpg)](https://youtu.be/9jSZm7FcqmE?t=0s)

## help

```
python3 sim.py -h

pygame 2.0.0.dev10 (SDL 2.0.12, python 3.7.6)
Hello from the pygame community. https://www.pygame.org/contribute.html
usage: sim.py [-h] [-a] [-d] [-f] [-s SPEED] [--bots-count BOTS_COUNT]
              [--bots-speed BOTS_SPEED]

DeepRacer Simulator

optional arguments:
  -h, --help            show this help message and exit
  -a, --autonomous      autonomous
  -d, --draw-lines      draw lines
  -f, --full-screen     full screen
  -s SPEED, --speed SPEED
                        speed
  --bots-count BOTS_COUNT
                        bots count
  --bots-speed BOTS_SPEED
                        bots speed
```
