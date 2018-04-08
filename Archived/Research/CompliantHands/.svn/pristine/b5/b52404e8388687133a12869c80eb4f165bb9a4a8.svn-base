#/usr/bin/env python

import os
import subprocess
for f in os.listdir("world"):
    if f.endswith(".xml"):
        print "Running reflex_col.py in batch mode on", f
        p = subprocess.Popen(["python", "reflex_col.py", "world/"+f], stdout=subprocess.PIPE)
        out, err = p.communicate() 
        result = out.split('\n')
        for lin in result:
            if not lin.startswith('#'):
                print(lin)
