
1/ Toujours ce probleme de "/" a la compilation

2/ lancement du hw : 

export ROBOT_TYPE=a1

roslaunch legged_ylo2_hw legged_ylo2_hw.launch  (demarre rviz, mais avec robot blanc)

roslaunch legged_controllers load_controller.launch (démarre les shared lib.so dans le 1er terminal)

rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0"

( cette commande affiche le robot correctement dans rviz, mais ne se déclence que quand les libs.so sont chargées.
  cependant, problemes de dummies dans rviz, une issue a été postée sur leur github.
  probleme : bien que le robot soit en initial pose et zeroé, les pattes cherchent d'autres joints que ceux du départ 
)

next soon.
