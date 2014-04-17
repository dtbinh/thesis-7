from os import system

import sys

if len(sys.argv) == 2:

    page_number = sys.argv[1]

elif len(sys.argv) == 1:

    page_number = '1'

system('pdflatex ekemper_defense_presentation.tex')

#system('bibtex main.aux')

system('pdflatex ekemper_defense_presentation.tex')

#system('pkill -3 evin')

system('evince --page-label='+ page_number +' ekemper_defense_presentation.pdf &')

#system('gnome-open main.pdf &')
