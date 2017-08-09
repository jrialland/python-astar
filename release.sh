

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

rm ./dist -rf

python2 setup.py sdist bdist_wheel
python3 setup.py sdist bdist_wheel

twine upload dist/*.whl

