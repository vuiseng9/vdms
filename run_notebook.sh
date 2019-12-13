#!/usr/bin/env bash

jupyter notebook --generate-config && \
        echo "c.NotebookApp.password='sha1:d12a2dd64552:86359144939d58019e6e5b37e373a50426ad7b2b' ">> /root/.jupyter/jupyter_notebook_config.py

mkdir -p /vdms-notebook
cd /vdms-notebook
git clone https://github.com/vuiseng9/vdms-examples
cd vdms-examples && git checkout deeplens-examples
cd ..
jupyter notebook --ip 0.0.0.0 --allow-root

