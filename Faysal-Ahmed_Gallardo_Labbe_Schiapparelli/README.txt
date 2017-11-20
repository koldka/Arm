This README file describes the ARM Project 2017 release.
This project was carried out for the Acquisition, Reconstruction and Modelisation classe.

Team members :
Faysal Ahmed Khaled
Gallardo Arnaud
Labbé Emeric
Schiapparelli Ugo

This project has been achieved using QT, OpenGL and C++11.

/* ---- Compilation and execution---- *\

To compile it, you can either run QTcreator and compile using Qt5 and C++11.

Or you can compile it in a shell, using qmake and make.
It will create an application named "Project".
Run it with ./Project

/* ---- Opening File ----*\

There is a toolbar  on the top of the window; to open an object click on "Fichier", "Ouvrir" and
select the file wanted.
We can open two file formats : .pgm3d and .obj

/* ---- Saving file ---- *\

We also implemented file saving. To save a file : "Fichier", "Sauvegarder" and
select the folder you want to save your data in.

We can change the transparancy using the slider on the left.

We can go from Surfacic to Volumic representation in the menu "Affichage".
Since we used one of those, you need to go back to "Personnalisé" to use the slider again.

/* ---- Controls ---- *\ 

Rotation can be done with the left click.
Use the wheel button to control the zoom.
Translations, following cardinal directions, are bind with keys:
Z :: Up
S :: Down
Q :: Left
D :: Right

/* ---- Improvement ---- *\

We could have avoided duplicating code. And we could have implemented a better data structure like an octree, 
to better manage the different components of a pgm3D.

Also, we are not using normals yet (We are reading them, but not using them).


