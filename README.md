# RBE550_project
 PRoject for RBE550 Motion Planning

#Building the environement
RUN `./build_docker_image.sh` this will build the image and place the fanuc repo in the correct location
 
#starting up the enviornment
Run `./docker_start.sh` this will start up a bash terminal inside of the docker conatiner.
Next `source devel/setup.bash` this allows to source the ros pathing this will need to be done everytime a next terminal is open.
If a more terminals are needed run `./docker_exec.sh` which will open another bash terminal inside the same conatiner that is already running
Finally to close a terminal `ctl+d` and to close the contatiner go to the othiganl bash terminal `clt+d`

# Adding any python dependencies
Simply add the depenedcy to requiremtn.txt 

# Launching demo 
To run moveit RVIZ demo in the terminal run `roslaunch fanuc_cr7ial_moveit_config demo.launch`

# VsCode
The container can be used as an environement to run code. To connect to the container use `Dev Containers` extension

