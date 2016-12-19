 From: Kevin Stein <Kevin.Stein@iwr.uni-heidelberg.de>
Subject: Trajectory
Date: Tue, 13 Dec 2016 11:52:24 +0100
To: Manuel Kudruss <manuel.kudruss@iwr.uni-heidelberg.de>

Hi,

hier schonmal die Daten vom Vorwärtslaufen.

Kurze Erklärung:
q_stream.csv:
Ergenbisse der Inveresen Kinematic

Alle anderen .csv:
Daten die über das Netzwerk ausgetauscht wurden, alle 20ms.
time_out.csv beinhaltet die Zeit, CoM, l_foot, r_foot, ZMP die Feedback 
informationen in World Coordinates (über die Floating Base der IK).

Die Reihenfolge ist:
x,y,z pos
x,y,z velocity (sehr ungenau)
x,y,z accelaration (unbrauchbar)

vel, und acc wurden mit finiten Differenzen berechnet, und über wenig 
Punkte gemittelt. Die Werte sind sehr verrauscht...

PatternGeneratorData.csv enthält alle Daten vom PG:
CoM Pos (x,y,z)
CoM angles (roll, pitch, yaw)
left_foot pos
left_foot angles
right_foot pos
right_foot angles
ZMP_reference pos


Die Daten mit .txt kommen direkt aus dem Programm, das den Roboter 
ansteuert und sind Messungen der Sensoren. Habe sie nur der 
Vollständigkeit wegen hinzugefügt.

Im Anhang noch das Python Notebook mit dem ich geplottet habe...

Viele Grüße
Kevin
