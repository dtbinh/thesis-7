from os import system

import sys

if len(sys.argv) == 2:

    page_number = sys.argv[1]

elif len(sys.argv) == 1:

    page_number = '1'

system('pdflatex main.tex')

system('bibtex main.aux')

system('pdflatex main.tex')

#system('pkill -3 evin')

system('evince --page-label='+ page_number +' main.pdf &')

#system('gnome-open main.pdf &')
