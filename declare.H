            //- Degrees to radians conversion factor.
            const scalar degRad=(Foam::constant::mathematical::pi)/180.0;

            //- Revolutions per second to radians per second conversion factor.
            const scalar rpsRadSec=2.0*(Foam::constant::mathematical::pi);

            //- Revolutions per minute to radians per second conversion factor.
            const scalar rpmRadSec=2.0*(Foam::constant::mathematical::pi)/60.0;

        //- Current time step size.
        scalar dt;

        //- Current simulation time.
        word time;
        scalar t;

        //- Boolean that is true past time step.
        bool pastFirstTimeStep;


        //- Turbine Array Level Data (all variables start with lower case letter).
            //- Body force vector field created by turbine array.
            //- List of names of turbines in array.
            List<word> turbineName;

            //- Number of turbines in array.
            int numTurbines;

	//- Write every "outputInterval" time steps or seconds.  Options are
        //  "timeStep" or "runTime".  "runTime" writes out as closely to every
	//  "outputInterval" seconds as possible, but doesn't adjust the time
        //  step to write on the exact interval. 
	word outputControl;

        //- The inteveral over which to write out turbine data.
        scalar outputInterval;

	//- Last time when output written.
	scalar lastOutputTime;

        //- Value to perturb blade locations by to break ties when searching for control processor.
        scalar perturb;

    	//- Last time step when output written.
	label outputIndex;

            //- List of names of turbine types in array.
            DynamicList<word> turbineType;
 
            //- List of locations of bases of turbines in array relative to origin (m).
            DynamicList<vector> baseLocation;

            //- List of number of actuator line points on blades of turbines
            //  in array. 
            DynamicList<int> numBladePoints; 

            //- List of description of actuator line point distribution types
            //  for each turbine (set to uniform for now--here for future upgrades).
            DynamicList<word> pointDistType;

            //- List of description of how velocity field is interpolated from the 
            //  CFD mesh to the actuator line points.  Options are "cellCenter" or
            //  "linear".  "cellCenter" uses the value at the cell center of the cell
            //  within which the actuator point lies.  "linear" uses linear
            //  interpolation from the cell within which the actuator point lies and
            //  from neighboring cells.
            DynamicList<word> pointInterpType;

            //- List of how the blades are updated in time.  "oldPosition" means that for
            //  computing the body force due to the blades for t^(n+1), the velocity is
            //  sampled from the location of the blades at time t^n.  The blades are advanced
            //  to their t^(n+1) position.  Then the blade force is computed at the updated  
            //  blade location and projected from there onto the flow field.  "newPosition"
            //  means that the blades are first advanced to thier t^(n+1) location, and then
            //  velocity is sampled there, blade forces are computed there, and body forces
            //  are projected.   
            DynamicList<word> bladeUpdateType;

            //- List of body force normalization parameter for each turbine (m). This controls
            //  the width of the Gaussian projection.  It should be tied to grid width.
            //  A value below 1 times the local grid cell length will yield inaccurate
            //  projection of the forces to the grid (i.e., if you integrate the projected
            //  force, it will be significantly smaller than the force that was projected
            //  in the first place.
            DynamicList<scalar> epsilon;

            //- List of tip/root loss correction type for each turbine.  "none" applies
            //  no correction.  "Glauert" applies the Glauert tip loss correction.
            DynamicList<word> tipRootLossCorrType;

	    //  "cw" for clockwise and "ccw" for counter-clockwise.
	    DynamicList<word> rotationDir;

            //- Initial or fixed rotor speed (rpm).  A positive value means
	    //  clockwise rotation for a clockwise rotating turbine (see rotationDir
	    //  above) or counter-clockwise rotation for a counter-clockwise
	    //  rotating turbine.
            DynamicList<scalar> rotSpeed;

            //- Filtered rotor speed that is fed to the controllers (rpm).
            DynamicList<scalar> rotSpeedF;

            //- Speed error between reference low speed shaft and filtered low speed
            //  shaft speed for use in blade pitch PID control (rad/s).
            DynamicList<scalar> speedError;

            //- Integrated speed error used in blade pitch PID control (rad/s).
            DynamicList<scalar> intSpeedError;

            //- Initial blade 1 azimuth angle (degrees) (looking from upwind to 
	    //  downwind, a positive azimuth angle makes a clockwise movement if
	    //  this is a clockwise rotating turbine (see rotationDir above) or
	    //  or a counterclockwise movement if this is a counter-clockwise
	    //  rotating turbine).
            DynamicList<scalar> azimuth;

            //- Initial generator torque on turbine (not density normalized).
            DynamicList<scalar> torqueGen;

            //- Initial blade pitch (degrees) of all blades.
            DynamicList<scalar> pitch;

            //- Initial or fixed nacelle yaw angle.  Direction that the turbine
            //  is pointed in cardinal directions (i.e. 0 = north, 90 = east, 
            //  180 = south, 270 = west) (degrees).  This is converted to radians
            //  in the more standard mathematical convention of 0 degrees on the 
            //  + x axis and positive degrees in the counter-clockwise direction.
            DynamicList<scalar> nacYaw;

            //- Specify the fluid density (kg/m^3).  This turbine model is to be  
            //  used with an incompressible solver, so density divides out of the 
            //  momentum equations.  Therefore, turbine forces are given to the 
            //  solver asforce/density.  To get actual forces, torques, and power 
            //  written to file, provide a density by which to multiply.
            DynamicList<scalar> fluidDensity;

            //- Number of distinct turbines in array.
            int numTurbinesDistinct;

            //- List of distinct names of turbine types in array.
            DynamicList<word> turbineTypeDistinct;

            //- ID label given to each distinct type of turbine in the array.
            DynamicList<label> turbineTypeID;

            //- Total actuator line points in array.
            int totBladePoints;

        //- Turbine Level Data (all variables start with a capital letter).

            //*** THE FOLLOWING VARIABLES MATCH FAST INPUT FILE ***
            //- Number of blades;
            DynamicList<int> NumBl;

            //- Distance from rotor apex to blade tip (m).
            DynamicList<scalar> TipRad;

            //- Distance from rotor apex to blade root (m).
            DynamicList<scalar> HubRad;

            //- Distance from teeter pin to rotor apex (m).
            DynamicList<scalar> UndSling;

            //- Distance from nacelle yaw axis to teeter pin or rotor apex (m).
            DynamicList<scalar> OverHang;

            //- Height of tower top above ground (m).
            DynamicList<scalar> TowerHt;

            //- Vertical distance from tower-top to rotor shaft centerline (m).
            DynamicList<scalar> Twr2Shft;

            //- Shaft tilt-up angle (degrees).
            DynamicList<scalar> ShftTilt;

            //- Coning angle of blades (degrees) (one for each blade).
            DynamicList<List<scalar> > PreCone;

            //- Gear-box ratio.
            DynamicList<scalar> GBRatio;

            //- Rated rotor speed (rpm).
            DynamicList<scalar> RatedRotSpeed;

            //- Moment of inertia of generator about high-speed shaft (kg-m^2).
            DynamicList<scalar> GenIner;

            //- Moment of inertia of hub about rotor shaft (kg-m^2).
            DynamicList<scalar> HubIner;
            //*** END OF FAST INPUT FILE VARIABLES ***

            //- Moment of inertia of a single blade about rotor shaft (kg-m^2).
            DynamicList<scalar> BladeIner;

            //- Moment of inertia of entire drivetrain (rotor blades, hub, and
            //  generator about the rotor shaft (kg-m^2).
            DynamicList<scalar> DriveTrainIner;

            //- Rotor speed controller type.  Options are "none" or "fiveRegion".
            //  "none" provides no torque control and the rotor rotates at a 
            //  constant rate specified by the variable "rotSpeed".  "fiveRegion"
            //  controls rotor speed through generator torque in regions 1, 1-1/2,
            //  2, 2-1/2, and 3.  Torque control alone will not control speed in
            //  region 3, but if no pitch control is implemented, then rotor speed
            //  will be limited at rated regardless of generator and rotor torque.
            DynamicList<word> GenTorqueControllerType;

	    //- Nacelle yaw controller type (NOT IMPLEMENTED, just remains at 
	    //  specified nacYaw).
	    DynamicList<word> NacYawControllerType;

	    //- Pitch controller type (NOT IMPLEMENTED, just remains at 
	    //  specified pitch).
	    DynamicList<word> BladePitchControllerType;

            //- Engage a rotor speed limiter (do not let rotor speed exceed rated
            //  or become negative.
            DynamicList<bool> RotSpeedLimiter;

            //- Engage a generator torque rate limiter.
            DynamicList<bool> GenTorqueRateLimiter;

            //- Engage a yaw rate limiter.
            DynamicList<bool> NacYawRateLimiter;

            //- Engage a blade pitch rate limiter.
            DynamicList<bool> BladePitchRateLimiter;

            //- Parameter for low-pass speed filter for control system (Hz).
            DynamicList<scalar> SpeedFilterCornerFrequency;

            //- Generator torque control parameters.
                //- Limiter on rate of change of generator torque (N-m/s).
                DynamicList<scalar> RateLimitGenTorque;

                //- Cut-in generator speed (rpm).
                DynamicList<scalar> CutInGenSpeed;

                //- Generator speed at start of region 2 (rpm).
                DynamicList<scalar> Region2StartGenSpeed;

                //- Generator speed at end of region 2 (rpm).
                DynamicList<scalar> Region2EndGenSpeed;

                //- Cut-in generator torque (N-m).
                DynamicList<scalar> CutInGenTorque;

                //- Rated generator torque (N-m).
                DynamicList<scalar> RatedGenTorque;

                //- Torque controller constant of proportionality (N-m/rpm^2).
                //  T_gen = K_gen*omega^2.
                DynamicList<scalar> KGen;


                //- Generator speed versus generator torque table (rpm, N-m).
                DynamicList<List<List<scalar> > > SpeedTorqueTable;
                DynamicList<DynamicList<scalar> > SpeedGenProfile;
                DynamicList<DynamicList<scalar> > TorqueGenProfile;

            //- Pitch control parameters.
                //- Maximum rate of change of blade pitch (degree/s).
                DynamicList<scalar> RateLimitBladePitch;

                //- Minimum pitch (degrees).
                DynamicList<scalar> PitchMin;

                //- Maximum pitch (degrees).
                DynamicList<scalar> PitchMax;

                //- Blade pitch angle at which the sensitivity of power to
                //  changes in blade pitch has doubled from its value at the
                //  rated operating point (degrees).
                DynamicList<scalar> PitchK;

                //- Pitch control gains.
                DynamicList<scalar> PitchControlKP; // (s)
                DynamicList<scalar> PitchControlKI; // (unitless)
                DynamicList<scalar> PitchControlKD; // (s^2)

            //- Nacelle yaw control parameters
                //- Maximum rate of change of nacelle yaw (degrees/s).
                DynamicList<scalar> RateLimitNacYaw;

            //- List of airfoils that compose turbine blade;
            DynamicList<List<word> > AirfoilType;


            //- Lists of blade data for each turbine type.
                //- Overall blade data array.
                DynamicList<List<List<scalar> > > BladeData;

                //- Station along blade in which information is given (m).
                DynamicList<DynamicList<scalar> > BladeStation;

                //- Chord at this station (m).
                DynamicList<DynamicList<scalar> > BladeChord;

                //- Twist at this station (degrees).
                DynamicList<DynamicList<scalar> > BladeTwist;

                //- Airfoil type ID at this station.
                DynamicList<DynamicList<label> > BladeAirfoilTypeID;

            //- List of distinct type of airfoils amongst all turbines in array.
            DynamicList<word> airfoilTypesDistinct;

            //- Overall airfoil data array.
            DynamicList<List<List<scalar> > > airfoilData;
            
            //- Angle-of-attack.
            DynamicList<DynamicList<scalar> > airfoilAlpha;

            //- Lift Coefficient.
            DynamicList<DynamicList<scalar> > airfoilCl;

            //- Drag Coefficient.
            DynamicList<DynamicList<scalar> > airfoilCd;

            //- List of locations of the intersection of the tower axis and the shaft 
            //  centerline relative to the origin (m).
            DynamicList<vector> towerShaftIntersect;

            //- List of locations of the rotor apex relative to the origin (m).
            DynamicList<vector> rotorApex;

            //- The projection of actuator forces to body forces uses a Gaussian (or some
            //  other smooth normalization could be used).  At some distance away from the
            //  actuator point from which the force is being projected, this normalization
            //  dies to 0.1% of its peak value.  Beyond that distance, stop doing the projection
            //  to save computational effort.  This variable is that distance in (m).  It
            //  is based on epsilon.  The larger epsilon, the wider the projection.
            DynamicList<scalar> projectionRadius;

        //- Important Actuator Line Geometry Data.
            //- List of turbines that this processor can forseeably control.
            DynamicList<label> turbinesControlled;

            //- List of list of labels or cells within sphere of action.
            DynamicList<DynamicList<label> > sphereCells;

	    //- An indicator of shaft direction.  The convention is that when viewed
            //  from upwind, the rotor turns clockwise for positive rotation angles,
            //  regardless of if it is an upwind or downwind turbine.  uvShaft is
            //  found by subtracting the rotor apex location from the tower shaft
            //  intersection point.  This vector switches direciton depending on 
            //  if the turbine is upwind or downwind, so this uvShaftDir multiplier
            //  makes the vector consistent no matter what kind of turbine.
	    DynamicList<scalar> uvShaftDir;

            //- Unit vector pointing along the rotor shaft (axis of blade rotation).
            DynamicList<vector> uvShaft;

	    //- Unit vector pointing along the tower (axis of yaw).
	    DynamicList<vector> uvTower;

            //- Actuator element width.
            DynamicList<DynamicList<scalar> > db;

            //- Actuator line point locations with respect to origin.
            DynamicList<List<List<vector> > > bladePoints;

            //- Blade radius away from rotor apex.  Must take into account coning.
            DynamicList<List<List<scalar> > > bladeRadius;

            //- Random perturbation of blade points applied only when determining control
            //  processors to break ties.  This does not affect true location where
            //  velocity is sampled and forces are applied.
            DynamicList<List<List<vector> > > bladePointsPerturbVector;

            //- Blade force at each actuator point.
            DynamicList<List<List<vector> > > bladeForce;

            //- Three vectors for each blade of each turbine that define the local
            //  blade-aligned coordinate system.  Vector 0 is along the blade pointed
            //  from root to tip, vector 1 is in the tangential direction (direction
            //  of blade rotation) where positive points in the direction opposite 
            //  rotation if the rotor turns clockwise as viewed from upstream, and 
            //  points orthogonal to vector 0 and 1 and points toward downstream (but 
            //  vector 2 is not perfectly aligned with downstream due to rotor coning  
            //  and nacelle tilt).
            DynamicList<List<List<vector> > > bladeAlignedVectors;

            //- Wind vector at each actuator point in blade-aligned coordinate system.
            DynamicList<List<List<vector> > > windVectors;

	    //- Change in yaw each time step.
	    DynamicList<scalar> deltaNacYaw;

	    //- Change in azimuth each time step.
	    DynamicList<scalar> deltaAzimuth;



        //- Information critical to turbine performance that can be written to file
        //  every time step.

            //- Angle of attack at each actuator point.
            DynamicList<List<List<scalar> > > alpha;

            //- Wind magnitude (not including radial wind) at each actuator point.
            DynamicList<List<List<scalar> > > Vmag;

            //- Coefficient of lift at each actuator point. 
            DynamicList<List<List<scalar> > > Cl;

            //- Coefficient of drag at each actuator point. 
            DynamicList<List<List<scalar> > > Cd;

            //- Lift/density at each actuator point. 
            DynamicList<List<List<scalar> > > lift;

            //- Drag/density at each actuator point. 
            DynamicList<List<List<scalar> > > drag;

            //- Axial force/density at each actuator point (not pointed in blade-local
            //  axial, but rather along shaft). 
            DynamicList<List<List<scalar> > > axialForce;

            //- Tangential force/density at each actuator point.
            DynamicList<List<List<scalar> > > tangentialForce;

            //- Thrust/density on turbine.
            DynamicList<scalar> thrust;

            //- Total rotor torque/density on turbine.
            DynamicList<scalar> torqueRotor;

            //- Power/density on turbine rotor.
            DynamicList<scalar> powerRotor;

            //- List of cell ID that contains a certain actuator line point on the
            //  processor in control of that point.  If the value is -1, then this
            //  processor is not in control of this point.
            DynamicList<List<List<label> > > minDisCellID;

        //- Output Data File Information.
            //- List of output files for angle of attack.
            OFstream* alphaFile_;

            //- List of output files for wind magnitude.
            OFstream* VmagFile_;

            //- List of output files for axial velocity.
            OFstream* VaxialFile_;

            //- List of output files for tangential velocity.
            OFstream* VtangentialFile_;

            //- List of output files for radial velocity.
            OFstream* VradialFile_;

            //- List of output files for coefficient of lift.
            OFstream* ClFile_;

            //- List of output files for coefficient of drag.
            OFstream* CdFile_;

            //- List of output files for lift/density.
            OFstream* liftFile_;

            //- List of output files for drag/density.
            OFstream* dragFile_;

            //- List of output files for axial force/density.
            OFstream* axialForceFile_;

            //- List of output files for tangential/density.
            OFstream* tangentialForceFile_;

            //- List of output files for actuator point locations.
            OFstream* xFile_;
            OFstream* yFile_;
            OFstream* zFile_;

            //- List of output files for total aerodynamic torque/density.
            OFstream* torqueRotorFile_;

            //- List of output files for generator torque/density.
            OFstream* torqueGenFile_;

            //- List of output files for total thrust/density.
            OFstream* thrustFile_;

            //- List of output files for total power/density.
            OFstream* powerRotorFile_;

            //- List of output files for rotation rate.
            OFstream* rotSpeedFile_;

            //- List of output files for filtered rotation rate.
            OFstream* rotSpeedFFile_;

            //- List of output files for blade 1 azimuth angle.
            OFstream* azimuthFile_;

            //- List of output files for blade pitch angle.
            OFstream* pitchFile_;

            //- List of output files for nacelle yaw angle.
            OFstream* nacYawFile_;

            //自由面数据文件
            OFstream* freesurfaceFile_;
