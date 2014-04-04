from os import system

import sys

if len(sys.argv) == 2:

    page_number = sys.argv[1]

elif len(sys.argv) == 1:

    page_number = '1'

system('pdflatex presentation_1.tex')

#system('bibtex main.aux')

system('pdflatex presentation_1.tex')

#system('pkill -3 evin')

system('evince --page-label='+ page_number +' presentation_1.pdf &')

#system('gnome-open main.pdf &')
