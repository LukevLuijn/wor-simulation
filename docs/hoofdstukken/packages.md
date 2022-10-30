
De applicatie bestaat uit twee nodes; arm_node (weergegeven als: '*custom_arm_node*') en de cup_node (weergegeven als: '*custom_cup_node*').

![packages](../assets/robot_sim_graph_02.svg)

**!diagram** - *ROS nodes & topics*

De arm_node luistert naar een topic genaamd */sim/controller/command*. Op dit topic kan de gebruiker van de applicatie en commando in de vorm van het AL5D protocol versturen. De arm_node reageert vervolgens op dit topic door joint_states te publiceren op het topic; */joint_states*.

De node; *robot_state_publisher* is een standaard node van ROS2 deze node luistert naar het eerder genoemde */joint_states* topic en genereert op basis van die waardes een 'robot_discription', deze wordt gepubliceerd op het topic */robot_description*.

Zodra blijkt dat de arm het kopje aan het oppakken is zal de arm een publicatie doen naar het */sim/arm/cup_pickup* topic. Zodra de waarde van dit topic true is zal de cup_node weten dat deze opgepakt is door de robotarm en zal de positie van het kopje aanpassen op de positie van de gripper van de robotarm.

Verder is de cup_node verantwoordelijk voor het publiceren van drie onderdelen, ten eerste de positie; de cup_node bepaald en publiceert zijn positie naar het topic */sim/cup/position*. De gebruiker kan deze positie vervolgens uitlezen via de commandline.

Het tweede onderdeel is de snelheid van het kopje. Het kopje bepaald zijn eigen snelheid ten opzichte van de wereld. De snelheid wordt gepubliceerd op het topic */sim/cup/speed*. Dit topic wordt vervolgens uitgelezen door RQT (weergegeven als: */rqt_gui_py_node_48138*) en de waardes geplot in de GUI.

Het derde en laatste onderdeel waar de cup_node verantwoordelijk voor is is het publiceren van een marker. De marker is een visuele weergaven van het daadwerkelijke kopje. De marker wordt naar het topic */sim/cup/marker* gepubliceerd.