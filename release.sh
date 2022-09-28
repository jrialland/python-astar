#! /bin/bash
set -euo pipefail

thisfile=$(readlink -m "$0")
thisdir=$(dirname "$thisfile")
pushd "$thisdir"

if [ ! -d .venv ]; then
    python -m venv .venv
fi

source .venv/bin/activate
pip install poetry


#some checks
poetry run mypy astar # check types
poetry run pytest # unit tests

#configure pypi
cat > ~/.pypirc << '_eof'
[distutils]
index-servers = pypi

[pypi]
repository:https://upload.pypi.org/legacy/
username:jrialland

[pypitest]
repository: https://test.pypi.org/legacy/
username: jrialland
_eof

# delete on exit
trap "rm ~/.pypirc" EXIT

rm ./dist -rf
poetry build
poetry run twine upload dist/*.whl

popd