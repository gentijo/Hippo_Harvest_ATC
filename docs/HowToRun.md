# How to Bring up the System and run the Simulator and Air traffic control
## Requirments:  Docker must be loaded on your system

## if you want telemetry and access via the MCP server
Run:

* git clone https://gitlab.com/open-telemetry/grafanastack.git
* cd grafanastack
* docker network create observe  (this typically only need to be done once, no harm to run more than once.)
* docker compose up -d

Once this system is up, you can launch your IDE and attach you code window the the Grafana MCP server. 
Just ask your IDE to let AI do it for you.

Now you should have a full Grafana stack + Open Telemetry collector and Tempo, Mimir, Loki


# To bring up the system (assumption this repo is already cloned)
if not
* git clone https://github.com/gentijo/Hippo_Harvest_ATC.git
* git checkout air_traffic_control_v1
* docker network create ros-net   (this typically only need to be done once, no harm to run more than once.)
( docker compose up -d

# now the base system is up and running
Run xhost + to allow the  docker container to access the host windowing system
open two terminals and run
* docker exec -it hippoharvest bash 

Run that command in both terminal windows, if it worked you should see the
prompt "root@hippoharvest"

**Run in terminal 1**

* cd /opt/code
* ./air_traffic_control/scripts/startup.sh

This should bring up rqt, in the plugin menu there will an entry Hippo HArvest, then under that is a module called "Air Traffic Control", run that plugin and you will see an empth status board

**Run in terminal 2**

* cd /opt/code
* ./simulation/scripts/run_nav2_rviz.sh

This will bring up rviz2, on the left hand pane you will see a small square, use your mouse to scroll in. 
The simulation will start running immedatlly and you should see the simulation running it RViz as well as
the ATC score board running and showing data in rqt.

The simulation will continue to run, untill you close the terminal window but robot activity will 
stop once the robots leave their home position, visit two random waypoints (red circles) then return home.

In the rqt screen there is a live "ATC Action" log at the bottom of the window.
This will show when colusions occure and show how they are cleared.

You can also open your browser and enter "localhost:3000" to bring up Grafana
From there you can inspect traces (Tempo data source) and logs (Loki data source)
