input = main.md
output = main.pdf

all:
	pandoc \
	--from markdown+implicit_figures \
	--table-of-contents \
	--highlight-style tango \
	-V lang=pl \
	-V geometry:margin=1.0in \
	-o main.pdf \
	main.md
