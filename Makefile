obj_to_adjmatrix : obj_to_adjmatrix.c
	gcc -O1 obj_to_adjmatrix.c -o obj_to_adjmatrix
obj_to_adjlist : obj_to_adjlist.c
	gcc -O1 obj_to_adjlist.c -o obj_to_adjlist
all : obj_to_adjmatrix obj_to_adjlist
