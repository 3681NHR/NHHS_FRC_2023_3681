#! /bin/sh

which python > /dev/null

if [ $? != 0 ]; then
  echo No python found in PATH\; exiting
  exit 1
fi

echo Using $(python --version)

python -m grpc_tools.protoc -I../roboRIO/src/main/proto --python_out=. --grpc_python_out=. ../roboRIO/src/main/proto/*

python main.py
