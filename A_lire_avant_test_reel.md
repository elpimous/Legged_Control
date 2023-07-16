
# legged_control  TODO list :

Avant démarrage en réel :

- modifier les configs des QDD100, en changeant le sens de rotation des moteurs (selon les polarités du vector motor_adapters_):
 1, 2, 3, 6, 7, 9
 
- simuler les contacts des foots : leggedcontroller.cpp L160

    contactFlag[i] = 1;


Oups, problemes rencontrés !!!  Mauvaise frame tx !!! vel & fft avaient les meme bytes !!! ### Résolu ! ###

-------------------------------------------------------------------------------------------------------------
1 / retirer les signs, ### Pas utile ! ###
2 / vérifier les frames TX, ### Vérifiées et ok ###
3 / Prendre en compte la polarité en pos & vel ### Fait ! ###
4 / tester si nécessaire les contactFlag en forcant à 1. ### Testé et pas de différence ! ###

A la demande de la commande suivante, :

rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0"

Mes pattes arrières sont bien positionnées, mais les 2 avant se lèvent via ABAD et pivotent vers l'avant.
???

Actuellement, simu ok, et hw compil ok.

si pas de démrrage avec la commande ci-dessus, utiliser : rosrun rqt_controller_manager rqt_controller_manager.

Donc, en attente de compréhension de ce probleme de démarrage !
