    if (Pstream::master())
    {
        // Create the name of the root of where turbine files get ouput.
        fileName rootDir;

        if (Pstream::parRun())
        {
            rootDir = runTime.path()/"../turbineOutput";
        }
        else
        {
            rootDir = runTime.path()/"turbineOutput";
        }

        // Check to see if the turbineOutput directory exists; if not, create it.    
        if (!isDir(rootDir))
        {
            mkDir(rootDir);
        }

        // Check to see if the start time directory exists within the turbineOutput directory; if not, create it.  
        if (!isDir(rootDir/time))
        {
            mkDir(rootDir/time);
        }



        // Create a total aerodynamic torque file.
        //torqueRotorFile(rootDir/time/"torqueRotor");
        torqueRotorFile_ = new OFstream(rootDir/time/"torqueRotor");
        *torqueRotorFile_ << "#Turbine    Time(s)    dt(s)    rotor torque (N-m)" << endl;

        // Create a generator torque file.
        torqueGenFile_ = new OFstream(rootDir/time/"torqueGen");
        *torqueGenFile_ << "#Turbine    Time(s)    dt(s)    generator torque (N-m)" << endl;

        // Create a total thrust file.
        thrustFile_ = new OFstream(rootDir/time/"thrust");
        *thrustFile_ << "#Turbine    Time(s)    dt(s)    thrust (N)" << endl;

        // Create a total power file.
        powerRotorFile_ = new OFstream(rootDir/time/"powerRotor");
        *powerRotorFile_ << "#Turbine    Time(s)    dt(s)    rotor power (W)" << endl;

        // Create a rotation rate file.
        rotSpeedFile_ = new OFstream(rootDir/time/"rotSpeed");
        *rotSpeedFile_ << "#Turbine    Time(s)    dt(s)    rotor rotation rate(rpm)" << endl;
        
        // Create a filtered rotation rate file.
        rotSpeedFFile_ = new OFstream(rootDir/time/"rotSpeedFiltered");
        *rotSpeedFFile_ << "#Turbine    Time(s)    dt(s)    filtered rotor rotation rate(rpm)" << endl;

        // Create a blade 1 azimuth angle file.
        azimuthFile_ = new OFstream(rootDir/time/"azimuth");
        *azimuthFile_ << "#Turbine    Time(s)    dt(s)    blade 1 azimuth angle (degrees)" << endl;

        // Create a blade pitch angle file.
        pitchFile_ = new OFstream(rootDir/time/"pitch");
        *pitchFile_ << "#Turbine    Time(s)    dt(s)    blade pitch angle (degrees)" << endl;

        // Create a nacelle yaw direction file.
        nacYawFile_ = new OFstream(rootDir/time/"nacYaw");
        *nacYawFile_ << "#Turbine    Time(s)    dt(s)    nacelle yaw angle (degrees)" << endl;

        // Create an angle of attack file.
        alphaFile_ = new OFstream(rootDir/time/"alpha");
        *alphaFile_ << "#Turbine    Blade    Time(s)    dt(s)    angle-of-attack(degrees)" << endl;

        // Create a wind speed magnitude file.
        VmagFile_ = new OFstream(rootDir/time/"Vmag");
        *VmagFile_ << "#Turbine    Blade    Time(s)    dt(s)    Vmag(m/s)" << endl;
    
        // Create an axial wind speed file.
        VaxialFile_ = new OFstream(rootDir/time/"Vaxial");
        *VaxialFile_ << "#Turbine    Blade    Time(s)    dt(s)    Vaxial(m/s)" << endl;

        // Create a tangential wind speed file.
        VtangentialFile_ = new OFstream(rootDir/time/"Vtangential");
        *VtangentialFile_ << "#Turbine    Blade    Time(s)    dt(s)    Vtangential(m/s)" << endl;

        // Create a radial wind speed file.
        VradialFile_ = new OFstream(rootDir/time/"Vradial");
        *VradialFile_ << "#Turbine    Blade    Time(s)    dt(s)    Vradial(m/s)" << endl;

        // Create a coefficient of lift file.
        ClFile_ = new OFstream(rootDir/time/"Cl");
        *ClFile_ << "#Turbine    Blade    Time(s)    dt(s)    Cl" << endl;

        // Create a coefficient of drag file.
        CdFile_ = new OFstream(rootDir/time/"Cd");
        *CdFile_ << "#Turbine    Blade    Time(s)    dt(s)    Cd" << endl;

        // Create a lift file.
        liftFile_ = new OFstream(rootDir/time/"lift");
        *liftFile_ << "#Turbine    Blade    Time(s)    dt(s)    lift (N)" << endl;

        // Create a drag file.
        dragFile_ = new OFstream(rootDir/time/"drag");
        *dragFile_ << "#Turbine    Blade    Time(s)    dt(s)    drag (N)" << endl;

        // Create a axial force file.
        axialForceFile_ = new OFstream(rootDir/time/"axialForce");
        *axialForceFile_ << "#Turbine    Blade    Time(s)    dt(s)    axial force (N)" << endl;

        // Create a tangential force file.
        tangentialForceFile_ = new OFstream(rootDir/time/"tangentialForce");
        *tangentialForceFile_ << "#Turbine    Blade    Time(s)    dt(s)    tangential force (N)" << endl;

        // Create a x-location file.
        xFile_ = new OFstream(rootDir/time/"x");
        *xFile_ << "#Turbine    Blade    Time(s)    dt(s)    x-location(m)" << endl;

        // Create a y-location file.
        yFile_ = new OFstream(rootDir/time/"y");
        *yFile_ << "#Turbine    Blade    Time(s)    dt(s)    y-location(m)" << endl;

        // Create a z-location file.
        zFile_ = new OFstream(rootDir/time/"z");
        *zFile_ << "#Turbine    Blade    Time(s)    dt(s)    z-location(m)" << endl;

        // 自由面数据
        freesurfaceFile_ = new OFstream(rootDir/time/"freesurface");
        *freesurfaceFile_ << "#Time(s)        x-location(m)        y-location(m)        z-location(m)        alpha.water" << endl;

    }


