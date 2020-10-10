--------------------------------------------------------------------------------
Author: Michael Wachl
Jahr: 2018
Kurs: Leistungskurs C++
Gruppe: 8
Universität: Technische Universität München

--------------------------------------------------------------------------------

# Objekterkennung
## Beschreibung Package/Idee
Dieses Package stellt den Objekterkennungs-Node dar. Dieser Node verwendet die 
Kinect und arbeitet mit Punktewolken. Die Punktewolken werden entspechend geclustert 
(k-d-Baum) und nach Objektdimensionen, Modellen (Zylinder und Fläche) und 
HSV-Werten gefiltert. Als Ergebnis werden nur Pfosten, Pucks und Turtles gepublished.
Die Objekterkennung arbeitet stabil, jedoch kann es zu Fehldetektionen außerhalb des
Spielfelds kommen. Diese Fehldetektionen werden dann aber vom Pfadplanungs-Node über die
bekannte Spielfelddimension gefiltert. Neben dem Lokalisierungs-Node verwendet 
auch der Roboteransteuerungs-Node ein Topic für die Puckaufnahme. Hierfür wird 
der nächstgelegene Puck der aktuellen Teamfarbe von dem Objekterkennungsnode
gepublished und vom Turtleansteuerungs-Node vearbeitet. 

Anfangs wurde allen Objekte in voller Größe gefiltert und geclustert. Dies 
hatte zur Folge, dass die Publish-Rate der Objekte auf dem Roboter bei ca. 0,7 Hz lag,
bei einer CPU auslastung von 140%. Deshalb hatte ich mich dazu entschieden nur einen 
Ausschnitt (ca. 7cm Streifen überhalb des Bodens) zu verwenden. Dies resultierte 
in einer Rate von 2 Hz bei ähnlicher Auslastung, jedoch mit mehr Fehldetektionen. 
Zum Schluss wurde noch die loop-Rate verringert was die Auslastung auf dem Roboter
auf ca. 70% begrenzt. 


#

--------------------------------------------------------------------------------
## Parameter
Alle Parameter sind in einem yaml-File gespeichert und können daher ohne neues komilieren
geändert werden. Zusätzlich sind die Parameter in Runtime über rqt_reconifgure 
veränderbar und als neues yaml-File exportierbar. Verwende dazu

`rosrun rqt_reconfigure  rqt_reconfigure`

Allgemein wurden alle Filterparameter relativ großzügig gewählt, damit die 
Objekterkennung immer noch bei sich ändernen Lichtverhältnissen stabil funktioniert.

#

--------------------------------------------------------------------------------
## Topics
Eigens definierte Array-Msg (siehe msg-Ordner) mit den detektierten Objekten

`/detected_objects`

Position des nächsten Pucks mit eigener Teamfarbe, als geometry_msgs::PointStamped msg

`/closest_puck`

--------------------------------------------------------------------------------
# Notwendige Libaries:
- standard ROS kinetic packages
- pcl libaries 
- Eigen3 libaries 


#

--------------------------------------------------------------------------------
# Launch
Die Bring-ups, Lokalisierungs-Node und Objekterkennungs-Node können mit einem 
Launchfile gestartet auf dem Turtlebot gestartet werden

`roslaunch wachl_3d_object_detection  turtlebot.launch`

um nur dieses Package zu launchen verwende

`roslaunch wachl_3d_object_detection  object_detection.launch`



