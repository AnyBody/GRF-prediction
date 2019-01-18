GROUND REACTION FORCE PREDICTION
================================

> **Note**: This guide is deprecated since Ground Reaction Prediction is now part of the AMMR. Please see the [MoCap GRF prediction example](https://anyscript.org/ammr-doc/auto_examples/Mocap/plot_Plug-in-gait_Simple_FullBody_GRFPrediction.html). 

## Introduction

![image](https://cloud.githubusercontent.com/assets/1038978/19969105/9a7834b6-a1d7-11e6-8880-26bb8fbc489e.png)

Motion capture data is often recorded without force plates. In traditional inverse dynamics, this would make it impossible to perform a dynamic analysis. However, AnyBody has the possibility to predict ground reaction forces (GRF), so you can make inverse dynamics models based on recorded motion without GRF force measurement (Fluit et al., 2014; Jung et al., 2014).

GRF prediction relies on conditional contacts added to the feet of the model. The conditional contacts work as force actuators to generate the normal and frictional forces necessary to balance model. Mathematically, the actuators are modeled similarly to muscles, and the muscle recruitment optimization determines the contact forces. The effect is that the model will utilize the ground reactions if that can minimize the muscle activity.

Adding conditional contacts to a model can be rather complex, but we have created an AnyScript class template that makes the process much easier. The class template will generate all the necessary AnyScript code. Thus, adding GRF prediction is similar to adding force plates to a model.

This new functionality is not yet released to the AnyBody Managed Model Repository (AMMR). The files needed for this to work must be downloaded separately.

The following section illustrates how to add the code to an existing mocap model.


## Adding ground force prediction to a mocap model.

### Including the class templates

> **Note**: This step is no longer necessary. As of AMMR v2.1  the Ground Reaction force class templates are part the model repository. See the [MoCap GRF prediction example](https://anyscript.org/ammr-doc/auto_examples/Mocap/plot_Plug-in-gait_Simple_FullBody_GRFPrediction.html). 

~~The following shows how to add ground reaction force prediction to a mocap example application from AMMR.~~

1. ~~Download the files from this folder and place them in the same directory as the main file.~~

![image](https://cloud.githubusercontent.com/assets/1038978/19969138/ba3a37f4-a1d7-11e6-9998-5b76ecaf4de2.png)

2. ~~Open the main file of your model and add the following to the top of the main file (MoCap_FullBody.main.any):~~
   ```c++
   // Include the classes for Ground Reaction Force Prediction
   #include "GRFPrediction/FootPlateConditionalContact.any"
   ```

### Removing the force plates
Next, we must delete the force plates which are used by default by the mocap model. 
Open the “MoCapModel\Model\Environment.any”  and comment line 20, which will exclude the force plates:

```c++
#define HumanModelPresent 1
// #include "../Input/ForcePlates.any"
#undef HumanModelPresent
```

### Adding the new GRF prediction classes

The next step is to use the class template that creates the AnyScript code for the GRF prediction. The `GRFPrediction/` directory you unzipped earlier also contains an example of such a file. So we are just going to use that.

We will add the code to the `ModelEnvironmentConnection` model folder, but the code should only be included in the Inverse Dynamic part of the model. So add the following (marked with red) around line 64 in the main file:

```c++
#if InverseDynamicModel
Main.Studies.InverseDynamicStudy.ModelEnvironmentConnection = {
    #include "GRFPrediction/GRF_example.any"
};
#include "Model/InverseDynamics.any"  
#endif
```

### Setting up new residuals (Hand of God)

We are going to look at the `GRF_example.any` file later, but adding conditional contacts to the feet is not enough. The mocap model comes with ‘hand of God’ reactions applied to the pelvis. These are the reactions that carry any inconsistencies between the model and force plate data, and they must be removed for ground reaction force prediction. Instead, we apply another type of residuals to the model, so the solver does not fail if the model fails to balance. These residuals are implemented as actuators from pelvis to the global reference frame. Mathematically, these actuators are also implemented similarly to how muscles work. However, they are very weak, so the recruitment solver will only activate them if nothing else can balance the model. The file `WeakResiduals.any` does the job of removing the ‘Hand of God’ and adding the new, weak residuals. Please make the following change in the main file (marked with red):


```c++
#if InverseDynamicModel
Main.Studies.InverseDynamicStudy.ModelEnvironmentConnection = {
  #include "GRFPrediction/GRF_example.any"
  #include "GRFPrediction/WeakResiduals.any"  
};
#include "Model/InverseDynamics.any"  
#endif
```
The steps above are all it takes add GRF prediction to the default Mocap model. The same code can be used for GRF 
prediction in any of the other models include in the AnyBody Managed Model Repository (AMMR).



### Running the model

The model is run in the same way as the Mocap model. The only difference is that it now uses GRF 
rediction instead of force plates data. The output of the now located under:
```
Main.Studies.InverseDynamicStudy.Output.ModelEnvironmentConnection.GRF_Prediction_Right
```

![image](https://cloud.githubusercontent.com/assets/1038978/19969209/082f83a6-a1d8-11e6-8ea7-7915bb817025.png)


It may be necessary to adjust the parameters of the GRF prediction class to obtain a good prediction 
of the ground reactions. This is especially important around heel strike and toe-off, where the model 
can have problems.  In the next section, we will look at what options that are directly available as 
settings to the GRF template. For a discussion of the limitations of this kind of GRF prediction, 
please refer to the following material:

1. Webcast:
  * Webcast on GRF prediction (www.anybodytech.com)
2. Wiki site with implementation details:
  * All about friction/contact actuators
3. Papers:
  * [Ground reaction force estimation using an insole-type pressure mat and joint kinematics during walking](http://www.sciencedirect.com/science/article/pii/S0021929014002930)
  * [Prediction of ground reaction forces and moments during various activities of daily living.](http://www.jbiomech.com/article/S0021-9290(14)00251-6/abstract)

### A closer look at the GRF template

Finally, we can take a look at the file GRF_example.any to see how conditional 
contact class is used. This is the file you will need to edit to add GRF 
prediction to your own model.

```c++
FootPlateConditionalContact GRF_Prediction_Right(
    NORMAL_DIRECTION = "Y",
    NUMBER_OF_NODES = 25,
    NODES_FOLDER = FootNodes,
    PLATE_BASE_FRAME = Main.EnvironmentModel.GlobalRef) =
{
    CreateFootContactNodes25 FootNodes(foot_ref = 
            Main.Studies.HumanModel.BodyModel.Right.Leg.Seg.Foot) = {};
};
```

It consists of two parts; a top level class template (`FootPlateConditionalContact`) that generate 
the conditional-contact code. This code needs a few important arguments. The ground plane (`PLATE_BASE_FRAME`), 
is a coordinate system where the ground plane is located. Together with arguments `NORMAL_DIRECTION` this 
specifies the surface the model is walking on. Another important argument is the `NODES_FOLDER`, which is a
 folder that contains all the contacts points.

The contact points can be created manually, but to avoid this we use another class-template (`CreateFootContactNodes25`)
to create the nodes automatically. As the name says it creates 25 nodes in the foot coordinate system. 
This part is specific to the model implementation. One could also imagine class-templates that produce a 
higher number of nodes or nodes in positions that corresponds to particular shoes etc.

Of course, there are also many options that can be tweaked and adjusted. The following documents the settings:

### Class templates: FootPlateConditionalContact

Obligatory members are marked with **< >** and optional values are marked with **[ ]**. Default values are **bold**:

```c++
FootPlateConditionalContact <ObjectName> (
       NORMAL_DIRECTION     = ["X"|"Y"|"**Z**"], 
       NUMBER_OF_NODES      = [**1**...200],
       NODES_BASE_FOLDER    = <AnyFolder> ,
       PLATE_BASE_FRAME     = <AnyRefFrame>,
       SHOW_TRIGGER_VOLUME = [**0**|1] ) =
{
    Settings =
    {
      [ LimitDistLow = **-0.10**; ]   
      [ LimitDistHigh = **0.04**;  ]
      [ LimitVelHigh = **0.8**;  ]
      [ Radius = **10**;  ]
      [ Strength = **200**; ]
      [ FrictionCoefficient = **0.5**;  ]
      [ ScaleFactor = **1**;   ]
      [ ForceVectorDrawScaleFactor = **0.0005**;] 
    };
 };    
 ```

#### Class arguments:
```
PLATE_BASE_FRAME:
    Reference to an AnyRefFrame object where the ground planes is attached.
NORMAL_DIRECTION:
    Defines the normal direction of the ground plane the in PLATE_BASE_FRAME coordinate system.
NODES_BASE_FOLDER :
    The folder where all contact nodes are located below.  contact nodes must be AnyRefNodes named must be named Node#   where # is a number. Eg. Node1 ...Node24
NUMBER_OF_NODES:
    The number of contact nodes to expect within NODE_BASE_FRAME
SHOW_TRIGGER_VOLUME:
    Debug option to visualize the volume where contacts may be triggered.
```

#### Optional settings:
```
   ------------------------
   Settings 
    .LimitDistLow                : 
    .LimitDistHigh               :
    .LimitVelHigh                :
    .Radius                      :
    .Strength                    :
    .FrictionCoefficient         :
    .ScaleFactor                 :
    .ForceVectorDrawScaleFactor  :
```

### Class template: CreateFootContactNodes25 
Creates 25 nodes in foot_ref that can be used for conditional contact. 
The nodes created are are created inside the subfolder `ConditionalContact_Nodes` 
and nodes are named nodes1… nodes25


Obligatory members are marked with **< >** 
```c++
CreateFootContactNodes25 <ObjectName> (
   foot_ref = <FootSegment>) =  
{ 
};
```

#### Class arguments:
foot_ref: 
	Reference to left or right foot segments. 


### References:
Fluit, R., Andersen, M.S., Kolk, S., Verdonschot, N., Koopman, H.F.J.M., 2014. Prediction of ground reaction forces and moments during various activities of daily living. J. Biomech. 47, 2321–2329.

Jung, Y., Jung, M., Lee, K., Koo, S., 2014. Ground reaction force estimation using an insole-type pressure mat and joint kinematics during walking. J. Biomech. 47, 2693–2699.
