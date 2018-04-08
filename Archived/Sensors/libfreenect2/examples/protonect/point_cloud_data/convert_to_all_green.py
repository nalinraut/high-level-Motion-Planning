
import sys

fi = open(sys.argv[1])
fo = open(sys.argv[2],'w')

green = "9.1477e-41"

for l in fi:
    fields = l.strip().split(" ")
    if len(fields)==4:
        fo.write(fields[0]+" "+fields[1]+" "+fields[2]+" "+str(green)+"\n")
    else:
        fo.write(l)
        
fi.close()
fo.close()

