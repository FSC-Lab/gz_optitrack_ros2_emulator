## Add the ground-truth plugin in the gazebo model
- To create an odometry publisher on a single link in a Gazebo model, in the sdf file of your PX4 SITL model

- Add to: ``/PX4_Autopilot/Tools/simulation/gz/models/MODEL_FOLDER/model.sdf``


```
<model>
…other stuff in sdf definition
    <!-- Observers -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <linkName>SUBMODEL_NAME::LINK_NAME</linkName>
      <odomTopic>odometry_groundtruth</odomTopic>
      <dimensions>3</dimensions>
      <odom_publish_frequency>100</odom_publish_frequency>  <!-- HZ -->
    </plugin>
…other stuff
</model>
</sdf>
```
