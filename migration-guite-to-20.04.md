
## Migration guide to Ubuntu 20.04

1. Upgrade you OS
2. Replace TriangleFunctor in osg code (I need to put a link to the corrected file here)
3. Install opencv headless wrappers for Python from Pypi: pip3 install opencv-python-headless. This avoids Qt issues with ViriatoPyRep
4. Remember to comment out the lines in Object.py  (PyRep local installation file)
