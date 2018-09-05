#!/usr/bin/python

import os, sys

"""
Generate a PDF file containaing one or more fiducial marker for printing
"""

def checkCmd(cmd, package):
    rc = os.system("which inkscape > /dev/null")
    if rc != 0:
        print """This utility requires %s. It can be installed by typing:
    sudo apt install %s""" % (cmd, package)
        sys.exit(1)

#   <rect x="26.0mm" y="42.0mm" width="150.0mm" height="4.0mm" style="stroke:black; fill:black"/>
#   <rect x="26.0mm" y="188.0mm" width="150.0mm" height="4.0mm" style="stroke:black; fill:black"/>
#   <rect x="26.0mm" y="42.0mm" width="4.0mm" height="150.0mm" style="stroke:black; fill:black"/>
#   <rect x="167.0mm" y="47.0mm" width="4.0mm" height="150.0mm" style="stroke:black; fill:black"/>

def genSvg(file, id, ids, dicno):
    f = open(file, "w")
    f.write("""<svg width="208.0mm" height="240.0mm"
 version="1.1"
 xmlns:xlink="http://www.w3.org/1999/xlink"
 xmlns="http://www.w3.org/2000/svg">

  <line x1="5.0mm" y1="5.0mm" x2="7.0mm" y2="5.0mm" style="stroke:black"/>
  <line x1="195.0mm" y1="5.0mm" x2="197.0mm" y2="5.0mm" style="stroke:black"/>
  <line x1="5.0mm" y1="21.0mm" x2="7.0mm" y2="21.0mm" style="stroke:black"/>
  <line x1="5.0mm" y1="21.0mm" x2="5.0mm" y2="23.0mm" style="stroke:black"/>
  <line x1="197.0mm" y1="21.0mm" x2="195.0mm" y2="21.0mm" style="stroke:black"/>
  <line x1="197.0mm" y1="21.0mm" x2="197.0mm" y2="23.0mm" style="stroke:black"/>
  
  <image x="26.0mm" y="42.0mm" width="150.0mm" height="150.0mm" xlink:href="marker%d.png" />

  <line x1="5.0mm" y1="213.0mm" x2="7.0mm" y2="213.0mm" style="stroke:black"/>
  <line x1="5.0mm" y1="213.0mm" x2="5.0mm" y2="211.0mm" style="stroke:black"/>
  <line x1="195.0mm" y1="213.0mm" x2="197.0mm" y2="213.0mm" style="stroke:black"/>
  <line x1="197.0mm" y1="213.0mm" x2="197.0mm" y2="211.0mm" style="stroke:black"/>
  <line x1="5.0mm" y1="229.0mm" x2="7.0mm" y2="229.0mm" style="stroke:black"/>
  <line x1="195.0mm" y1="229.0mm" x2="197.0mm" y2="229.0mm" style="stroke:black"/>

  <text x="90.0mm" y="220.0mm" style="font-family:ariel; font-size:24">[%d, %d, %d, %d] D%d</text>

</svg>
""" % (id, ids[0], ids[1], ids[2], ids[3], dicno))
    f.close()

if __name__ == "__main__":
    checkCmd("inkscape", "inkscape")
    checkCmd("pdfunite", "poppler-utils")

    dicno = 7
    argc = len(sys.argv)
    if argc != 3 and argc != 4:
        print "Usage: %s Ids pdfFile [dictionary]" % sys.argv[0]
        print('Ids examples:')
        print('1,2,3,4')
        print('1,2,3,4-7')
        print('1-5,2,3,4-7')
        print('All possible combinations will be created.')
        sys.exit(1)
    outfile = sys.argv[2]
    if argc == 5:
        dicno = int(sys.argv[3])
    ids_string = sys.argv[1].split(',')
    ids_pos = []
    for id_string in ids_string:
        ranges = id_string.split('-')
        if len(ranges) == 1:
            ids_pos.append([int(ranges[0])])
        else:
            r0 = int(ranges[0])
            r1 = int(ranges[1])
            if r0 > r1:
                r0, r1 = r1, r0
            ids_pos.append(list(range(r0, r1 + 1)))
    all_ids = []
    for id0 in ids_pos[0]:
        for id1 in ids_pos[1]:
            for id2 in ids_pos[2]:
                for id3 in ids_pos[3]:
                    all_ids.append([id0, id1, id2, id3])
    
    pdfs = ['marker%d.pdf' % i for i in range(len(all_ids))]

    for i, ids in enumerate(all_ids):
        print " Marker [%d, %d, %d, %d]\r" % tuple(ids),
        sys.stdout.flush()
        os.system("rosrun aruco_detect create_diamond_marker --sl=2000 --ml=1200 --ids=%d,%d,%d,%d --d=%d marker%d.png" % (tuple(ids) + (dicno, i,)))
        genSvg("marker%d.svg" % i, i, ids, 7)
        os.system("inkscape --without-gui --export-pdf=marker%d.pdf marker%d.svg" % (i, i))
        os.remove("marker%d.svg" % i)
        os.remove("marker%d.png" % i)
    print "Combining into %s" % outfile
    os.system("pdfunite %s %s" % (" ".join(pdfs), outfile))
    for f in pdfs:
        os.remove(f)
