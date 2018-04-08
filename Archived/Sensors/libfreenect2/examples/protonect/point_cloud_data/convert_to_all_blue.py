
import sys

fi = open(sys.argv[1])
fo = open(sys.argv[2],'w')

blue = 255;

for l in fi:
    fields = l.strip().split(" ")
    if len(fields)==4:
        fo.write(fields[0]+" "+fields[1]+" "+fields[2]+" "+str(blue)+"\n")
    else:
        fo.write(l)
        
fi.close()
fo.close()

