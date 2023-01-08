#/bin/bash -ex

motion_pid=$(pgrep motion)
if [ -z $motion_pid ]; then
    motion -c motion.conf
fi

source $HOME/py39/bin/activate
python run.py
