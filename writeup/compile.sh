#!/bin/bash

rm main.dvi
rm main.pdf
latex main 
bibtex main 
latex main 
latex main 
pdflatex main 
#open -a Preview main.pdf

