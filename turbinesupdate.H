    // Update the time step size.
    dt = runTime.deltaT().value();
    deltaAzimuth = rotSpeed * dt;

    // Update the current simulation time.
    time = runTime.timeName();
    t = runTime.value();

    if(bladeUpdateType[0] == "oldPosition")
    {
        // Find out which processor controls which actuator point,
        // and with that informatio sample the wind at the actuator
        // points.

    // Create a local and global list of minimum distance cells to actuator line 
    // points of turbines that this processor controls.  Initialize the values to huge.
    List<scalar> minDisLocal(totBladePoints,1.0E30);
    List<scalar> minDisGlobal(totBladePoints,1.0E30);

    forAll(turbinesControlled, p)//对水轮机的每个叶片都进行下文中的计算
    {
        int i = turbinesControlled[p];
        int iter = 0;//定义制动点总数
        if(i > 0)
        {
            for(int n = 0; n < i; n++)
            {
                iter += numBladePoints[n] * NumBl[turbineTypeID[n]];
            }
        }
        
        forAll(bladePoints[i], j)
        {
            forAll(bladePoints[i][j], k)
            {
                // Find the cell that the actuator point lies within and the distance
                // from the actuator line point to that cell center.
                label cellID = sphereCells[i][0];//仅仅使用将水轮机附近的点代入计算，最小距离中计入了扰动项的影响
                scalar minDis = mag(mesh.C()[cellID] - (bladePoints[i][j][k] + bladePointsPerturbVector[i][j][k]));

                forAll(sphereCells[i], m)//用循环算法找出距离距离叶片最近的网格
                {
                    scalar dis = mag(mesh.C()[sphereCells[i][m]] - (bladePoints[i][j][k] + bladePointsPerturbVector[i][j][k]));
                    if(dis <= minDis)
                    {
                        cellID = sphereCells[i][m];
                    }
                    minDis = mag(mesh.C()[cellID] - (bladePoints[i][j][k] + bladePointsPerturbVector[i][j][k]));
                }
                minDisLocal[iter] = minDis;
                minDisGlobal[iter] = minDis;
                minDisCellID[i][j][k] = cellID;
                iter++;
            }
        }
    }

    // Parallel gather/scatter the global minimum distance list and reduce it by keeping 
    // only the minimum values.
    Pstream::gather(minDisGlobal,minOp<List<scalar> >());//将所有并行的处理器中的值放到一起过滤出最小值
    Pstream::scatter(minDisGlobal);

    // Compare the global to local lists.  Where the lists agree, this processor controls
    // the actuator line point.
    forAll(turbinesControlled, p)
    {
        int i = turbinesControlled[p];
        int iter = 0;
        if(i > 0)
        {
            for(int n = 0; n < i; n++)
            {
                iter += numBladePoints[n] * NumBl[turbineTypeID[n]];
            }
        }
        
        forAll(bladePoints[i], j)
        {
            forAll(bladePoints[i][j], k)
            {
                if(minDisGlobal[iter] != minDisLocal[iter])
                {
                    minDisCellID[i][j][k] = -1;
                }
                iter++;
            }
        }
    }

    // Create a list of wind velocity in x, y, z coordinates for each blade point.
    List<vector> windVectorsLocal(totBladePoints,vector::zero);

    // If linear interpolation of the velocity from the CFD mesh to the actuator
    // points is used, we need velocity gradient information.
    gradU = fvc::grad(U);

    forAll(turbinesControlled, p)//通过线性差值或者不插值得到制动点处网格的风速
    {
        int i = turbinesControlled[p];
        int iter = 0;
        if(i > 0)
        {
            for(int n = 0; n < i; n++)
            {
                iter += numBladePoints[n] * NumBl[turbineTypeID[n]];
            }
        }
        
        forAll(bladePoints[i], j)
        {
            forAll(bladePoints[i][j], k)
            {
                if(minDisCellID[i][j][k] != -1)
                {
                    // If the velocity interpolation is "cellCenter", then just use 
                    // the velocity at the center of the cell within which this
                    // actuator point lies
                    windVectorsLocal[iter] = U[minDisCellID[i][j][k]];

                    // But if linear interpolation is used, add a correction based
                    // on the local velocity gradient.
                    if (pointInterpType[i] == "linear")
                    {
                        vector dx = bladePoints[i][j][k] - mesh.C()[minDisCellID[i][j][k]];
                        vector dU = dx & gradU[minDisCellID[i][j][k]];
                        windVectorsLocal[iter] += dU;
                    }
                }
                iter++;
            }
        }
    }

    // Perform a parallel gather of this local list to the master processor and
    // and then parallel scatter the list back out to all the processors.
    Pstream::gather(windVectorsLocal,sumOp<List<vector> >());//与并行有关的数据传递
    Pstream::scatter(windVectorsLocal);

    // Put the gathered/scattered wind vectors into the windVector variable.
    // Proceed turbine by turbine.
    int iter = 0;
    forAll(windVectors, i)//将每个网格点的风速值写入到风速表里面
    {
        // Proceed blade by blade.
        forAll(windVectors[i], j)
        { 
            // Proceed point by point.
            forAll(windVectors[i][j], k)
            {
                // Zero the wind vector.
                windVectors[i][j][k] = vector::zero;

                // Now put the velocity in that cell into blade-oriented coordinates.
                windVectors[i][j][k] = windVectorsLocal[iter];

                iter++;
            }
        }
    }


         // Update the rotor state.
//        filterRotSpeed();

    // Proceed turbine by turbine.
    forAll(torqueGen, i)
    {
        // Get the turbine type index.
        int j = turbineTypeID[i];

        // Get the current filtered generator speed.
        scalar genSpeedF = (rotSpeedF[i]/rpmRadSec)*GBRatio[j];


        // Initialize the commanded generator torque variable;
        scalar torqueGenCommanded = torqueGen[i];



        // Apply a controller to update the rotor speed.
	if (GenTorqueControllerType[j] == "none")
        {
            #include "controllers/genTorqueControllers/none.H"
        }

	else if (GenTorqueControllerType[j] == "fiveRegion")
        {
            #include "controllers/genTorqueControllers/fiveRegion.H"
	}

        else if (GenTorqueControllerType[j] == "speedTorqueTable")
        {
            #include "controllers/genTorqueControllers/speedTorqueTable.H"
        }

        // Limit the change in generator torque.
        if (GenTorqueRateLimiter[j])
        {
            #include "limiters/genTorqueRateLimiter.H"
        }

        // Update the pitch array.
        torqueGen[i] = torqueGenCommanded;
    }

    // Proceed turbine by turbine.控制叶片螺距
    forAll(pitch, i)
    {

        // Get the turbine type index.
        int j = turbineTypeID[i];
        
        // Initialize the gain scheduling variable.
        scalar GK = 0.0;

        // Initialize the commanded pitch variable.
        scalar pitchCommanded = pitch[i]*degRad;


        // Apply a controller to update the blade pitch position.
        if (BladePitchControllerType[j] == "none")
        {
            #include "controllers/bladePitchControllers/none.H"
        }

        else if (BladePitchControllerType[j] == "PID")
        {
            #include "controllers/bladePitchControllers/PID.H"
        }

        // Apply pitch rate limiter.
        if (BladePitchRateLimiter[j])
        {
            #include "limiters/bladePitchRateLimiter.H"
        }

        // Update the pitch array.
        pitch[i] = pitchCommanded/degRad;
    }

    // Proceed turbine by turbine.
    forAll(deltaNacYaw, i)
    {
        // Get the turbine type index.
        int j = turbineTypeID[i];


        
        // Apply a controller to update the nacelle yaw position.
        if (NacYawControllerType[j] == "none")
        {
            // Do nothing.
	    deltaNacYaw[i] = 0.0;
        }

        else if (NacYawControllerType[j] == "simple")
        {
            // Placeholder for when this is implemented.
        }
        
        else if (NacYawControllerType[j] == "timeYawTable")
        {
        }


        
        // Limit the change in nacelle yaw angle.
        if (NacYawRateLimiter[j])
        {
        }

    }

    // Proceed turbine by turbine.
    forAll(rotSpeed, i)
    {
        // Get the turbine type index.
        int j = turbineTypeID[i];

        // If the generator torque and blade pitch controllers are both set to "none", then
        // the rotor speed will remain fixed at its initial speed.
        if ((GenTorqueControllerType[j] == "none") && (BladePitchControllerType[j] == "none"))
        {
            // Do nothing.
        }

        // Otherwise numerically solve the dynamics of the rotor to compute the new rotor speed
        // based on the summation of aerodynamic and generator torque on the rotor.
        else
        {
            rotSpeed[i] += (dt/DriveTrainIner[j])*(torqueRotor[i]*fluidDensity[i] - GBRatio[j]*torqueGen[i]);
        }


        // Limit the rotor speed to be positive and such that the generator does not turn
        // faster than rated.
        if (RotSpeedLimiter[j])
        {
            # include "limiters/rotSpeedLimiter.H"
        }
 
        // Compute the change in blade azimuth angle based on the time step and current rotor speed.
        deltaAzimuth[i] = rotSpeed[i] * dt;

    }

    // Perform rotation turbine by turbine.
    forAll(uvShaft, i)
    {	
        // Check the rotation direction first and set the local delta azimuth
	// variable accordingly.
	scalar deltaAzimuthI = 0.0;//定义标量旋转角度
	if (rotationDir[i] == "cw")//判断旋转方向，cw为顺时针
	{
            deltaAzimuthI =  deltaAzimuth[i];
        }
	if (rotationDir[i] == "ccw")
	{
            deltaAzimuthI = -deltaAzimuth[i];
        }

	// Rotate turbine blades, blade by blade, point by point.
	forAll(bladePoints[i], j)//对所有叶片坐标点进行旋转操作，叶片点新坐标=旋转点函数（叶片旧坐标，旋转中心，轴向单位向量，旋转角度）
        {
            forAll(bladePoints[i][j], k)
            {
                bladePoints[i][j][k] = rotatePoint(bladePoints[i][j][k], rotorApex[i], uvShaft[i], deltaAzimuthI);
            }
        }   


	// Calculate the new azimuth angle and make sure it isn't
        // bigger than 2*pi.
        if (pastFirstTimeStep)
        {
	    azimuth[i] = azimuth[i] + deltaAzimuth[i];//更新转角，并将转角控制在2pi以内
            if (azimuth[i] >= 2.0 * Foam::constant::mathematical::pi)
            {
                azimuth[i] -= 2.0 * Foam::constant::mathematical::pi;
            }
        }
    }


//        yawNacelle();
/*
    //依次对每个风机进行偏航控制操作
    forAll(uvTower, i)
    {
	//将叶轮中心进行偏航旋转，叶轮中心新坐标=旋转点函数（叶轮中心旧坐标，旋转中心(塔架与机舱中轴交点)，转轴(塔架方向向量)，旋转角度）
        rotorApex[i] = rotatePoint(rotorApex[i], towerShaftIntersect[i], uvTower[i], deltaNacYaw[i]);

	//更新叶轮轴向单位向量
	uvShaft[i] = rotorApex[i] - towerShaftIntersect[i];
	uvShaft[i] = (uvShaft[i]/mag(uvShaft[i])) * uvShaftDir[i];
	
	//将所有叶片上的制动点坐标进行更新
	forAll(bladePoints[i], j)
        {
            forAll(bladePoints[i][j], k)
            {
                bladePoints[i][j][k] = rotatePoint(bladePoints[i][j][k], towerShaftIntersect[i], uvTower[i], deltaNacYaw[i]);
            }
        }   

	//跟新偏航角，并将偏航角控制在2pi以内
        if (pastFirstTimeStep)
        {
	    nacYaw[i] = nacYaw[i] + deltaNacYaw[i];
            if (nacYaw[i] >= 2.0 * Foam::constant::mathematical::pi)
            {
                nacYaw[i] -= 2.0 * Foam::constant::mathematical::pi;
            }
        }
    }
*/
    }
    else if(bladeUpdateType[0] == "newPosition")
    {
        // Update the rotor state.
/*        filterRotSpeed();
        controlGenTorque();
        controlBladePitch();
        controlNacYaw();
        computeRotSpeed();
        rotateBlades();
        yawNacelle();

        // Find out which processor controls which actuator point,
        // and with that information sample the wind at the actuator
        // points.
        findControlProcNo();
        computeWindVectors();
*/    }

    // Compute the blade forces.
    // Take the x,y,z wind vectors and project them into the blade coordinate system.
    // Proceed turbine by turbine.
    forAll(windVectors, i)
    {
        int n = turbineTypeID[i];

        // Proceed blade by blade.
        forAll(windVectors[i], j)//一步步创建径向单位向量[2]，切向单位向量[1]，轴向单位向量[0]
        {
            // If clockwise rotating, this vector points along the blade toward the tip.
	    // If counter-clockwise rotating, this vector points along the blade toward the root.
	    if (rotationDir[i] == "cw")
	    {
                bladeAlignedVectors[i][j][2] =   bladePoints[i][j][0] - rotorApex[i];
                bladeAlignedVectors[i][j][2] =   bladeAlignedVectors[i][j][2]/mag(bladeAlignedVectors[i][j][2]);
	    }
	    else if (rotationDir[i] == "ccw")
	    {
                bladeAlignedVectors[i][j][2] = -(bladePoints[i][j][0] - rotorApex[i]);
                bladeAlignedVectors[i][j][2] =   bladeAlignedVectors[i][j][2]/mag(bladeAlignedVectors[i][j][2]);
	    }

            // This vector points in the tangential direction opposite the turbines rotation type.  It is 
            // set up this way because it will point in the direction of oncoming flow that the blade sees 
            // due to rotation.
            bladeAlignedVectors[i][j][1] = bladeAlignedVectors[i][j][2]^uvShaft[i];
            bladeAlignedVectors[i][j][1] = bladeAlignedVectors[i][j][1]/mag(bladeAlignedVectors[i][j][1]);

            // This vector points normal to the other two and toward downwind (not exactly downwind if
            // the blade is coned).  It points in the direction of the oncoming flow due to wind that the
            // blade sees.
            bladeAlignedVectors[i][j][0] = bladeAlignedVectors[i][j][1]^bladeAlignedVectors[i][j][2];
            bladeAlignedVectors[i][j][0] = bladeAlignedVectors[i][j][0]/mag(bladeAlignedVectors[i][j][0]);
            
            // Proceed point by point.
            forAll(windVectors[i][j], k)//在风速备份后，将风速投影到沿着叶片建立的随体坐标系上，便于后面计算升阻力
            {
                windVectors[i][j][k].x()+=Vflow;
                vector windVectorsInt = windVectors[i][j][k];

                // Zero the wind vector.
                windVectors[i][j][k] = vector::zero;

                // Now put the velocity in that cell into blade-oriented coordinates and add on the
                // velocity due to blade rotation.
                windVectors[i][j][k].x() = (bladeAlignedVectors[i][j][0] & windVectorsInt);
                windVectors[i][j][k].y() = (bladeAlignedVectors[i][j][1] & windVectorsInt) + (rotSpeed[i] * bladeRadius[i][j][k] * Foam::cos(PreCone[n][j]));//切向考虑到叶片旋转的影响
                windVectors[i][j][k].z() = (bladeAlignedVectors[i][j][2] & windVectorsInt);
            }
        }
    }



    // Proceed turbine by turbine.
    forAll(windVectors, i)
    {
        int m = turbineTypeID[i];

        // Set the total thrust of the turbine to zero.  Thrust will be summed on a blade-element-
        // wise basis.
        thrust[i] = 0.0;

        // Set the total aerodynamic torque of the turbine to zero.  Thrust will be summed on a blade-element-
        // wise basis.
        torqueRotor[i] = 0.0;

        // Proceed blade by blade.
        forAll(windVectors[i], j)
        {

            // Proceed point by point.
            forAll(windVectors[i][j], k)
            {
                // Interpolate the local twist angle.
                scalar twistAng = interpolate(bladeRadius[i][j][k], BladeStation[m], BladeTwist[m]);//叶片倾角的插值

                // Interpolate the local chord.
                scalar chord = interpolate(bladeRadius[i][j][k], BladeStation[m], BladeChord[m]);//叶片弦长的插值

                // Find the local airfoil type.
                label airfoil = interpolate(bladeRadius[i][j][k], BladeStation[m], BladeAirfoilTypeID[m]);//对翼型的插值
                label maxIndex = BladeAirfoilTypeID[m].size() - 1;
                airfoil = min(max(0,airfoil),maxIndex);

                // Find the local velocity magnitude compose of only the axial and tangential flow (do
                // not include the radial (along blade span) flow).
                Vmag[i][j][k] = Foam::pow((Foam::pow(windVectors[i][j][k].x(),2) + Foam::pow(windVectors[i][j][k].y(),2)),0.5);//计算来流风速（忽略径向）

                // Get the angle of the wind with respect to rotor plane tangent direction.
                scalar windAng = Foam::atan2(windVectors[i][j][k].x(),windVectors[i][j][k].y())/degRad; //计算来流角度

                // Angle of attack is local angle of wind with respect to rotor plane tangent minus local twist.//计算攻角
                alpha[i][j][k] = windAng - twistAng - pitch[i];

                // Use airfoil look-up tables to get coefficient of lift and drag.
                Cl[i][j][k] = interpolate(alpha[i][j][k], airfoilAlpha[airfoil], airfoilCl[airfoil]);//获取升力系数
                Cd[i][j][k] = interpolate(alpha[i][j][k], airfoilAlpha[airfoil], airfoilCd[airfoil]);//获取阻力系数

                // Apply tip/root-loss correction factor.
                // Tip/root-loss correction factor of Glauert.
                scalar F = 1.0;

                if(tipRootLossCorrType[i] == "none")
                {
                    F = 1.0;
                }

                else if(tipRootLossCorrType[i] == "Glauert")//进行Glauert修正，即在大推力状态(a>0.4或CT>0.96F)时对轴向诱导因子 a 进行修正
                {
                    scalar g = 1.0;

                    scalar ftip  = (TipRad[m] - bladeRadius[i][j][k])/(bladeRadius[i][j][k] * Foam::sin(windAng*degRad));
                    scalar Ftip  = (2.0/(Foam::constant::mathematical::pi)) * Foam::acos(Foam::exp(-g * (NumBl[m] / 2.0) * ftip));

                    scalar froot = (bladeRadius[i][j][k] - HubRad[i])/(bladeRadius[i][j][k] * Foam::sin(windAng*degRad));
                    scalar Froot = (2.0/(Foam::constant::mathematical::pi)) * Foam::acos(Foam::exp(-g * (NumBl[m] / 2.0) * froot));

                    F = Ftip * Froot;
                }

                // Using Cl, Cd, wind velocity, chord, and actuator element width, calculate the
                // lift and drag per density.
                //lift[i][j][k] = 0.5 * F * Cl[i][j][k] * Vmag[i][j][k] * Vmag[i][j][k] * chord * db[i][k];
                //drag[i][j][k] = 0.5 * F * Cd[i][j][k] * Vmag[i][j][k] * Vmag[i][j][k] * chord * db[i][k];
                Cl[i][j][k] *= F;
                Cd[i][j][k] *= F;
                lift[i][j][k] = 0.5 * Cl[i][j][k] * Vmag[i][j][k] * Vmag[i][j][k] * chord * db[i][k];//根据公式计算升力（不可压）
                drag[i][j][k] = 0.5 * Cd[i][j][k] * Vmag[i][j][k] * Vmag[i][j][k] * chord * db[i][k];//根据公式计算阻力（不可压）

                // Make the scalar lift and drag quantities vectors in the Cartesian coordinate system.
                vector dragVector = bladeAlignedVectors[i][j][0]*windVectors[i][j][k].x() + bladeAlignedVectors[i][j][1]*windVectors[i][j][k].y();//只考虑轴向与切向的阻力（其实我觉得是阻力的逆向）
                dragVector = dragVector/mag(dragVector);

                vector liftVector = dragVector^bladeAlignedVectors[i][j][2];//升力的方向
                liftVector = liftVector/mag(liftVector);

                liftVector = -lift[i][j][k] * liftVector;//得到升力矢量
                dragVector = -drag[i][j][k] * dragVector;//得到阻力矢量

                // Add up lift and drag to get the resultant force/density applied to this blade element.
                bladeForce[i][j][k] = liftVector + dragVector;//得到合力

                // Find the component of the blade element force/density in the axial (along the shaft)
                // direction.
                axialForce[i][j][k] = -bladeForce[i][j][k] & uvShaft[i];//合力在轴向的投影

                // Find the component of the blade element force/density in the tangential (torque-creating)
                // direction.
                tangentialForce[i][j][k] = bladeForce[i][j][k] & bladeAlignedVectors[i][j][1];//合力在切向的投影

                // Add this blade element's contribution to thrust to the total turbine thrust.
                thrust[i] += axialForce[i][j][k];//加和推力

                // Add this blade element's contribution to aerodynamic torque to the total turbine aerodynamic torque.
                torqueRotor[i] += tangentialForce[i][j][k] * bladeRadius[i][j][k] * Foam::cos(PreCone[m][j]);//加和扭矩
            }
        }

        // Compute rotor power based on aerodynamic torque and rotation speed.
        powerRotor[i] = torqueRotor[i] * rotSpeed[i];//计算功率
    }

    // Project the blade forces as body forces.
    // Zero out the body force to begin with.
    bodyForce *= 0.0;

    // Proceed turbine by turbine.
    scalar thrustSum = 0.0;
    scalar torqueSum = 0.0;
    scalar thrustBodyForceSum = 0.0;
    scalar torqueBodyForceSum = 0.0;

    forAll(bladeForce, i)
    {
        int n = turbineTypeID[i];

        // Proceed to compute body forces for turbine i only if there are sphere cells on this processor for this turbine.
        if (sphereCells[i].size() > 0)
        {
            // For each blade.
            forAll(bladeForce[i], j)
            {
                // For each blade point.
                forAll(bladeForce[i][j], k)
                {
                    // For each sphere cell.
                    forAll(sphereCells[i], m)//对所有在计算范围内的网格进行高斯投影
                    {
                        scalar dis = mag(mesh.C()[sphereCells[i][m]] - bladePoints[i][j][k]);
                        if (dis <= projectionRadius[i])
                        {
                            bodyForce[sphereCells[i][m]] += bladeForce[i][j][k] * (Foam::exp(-Foam::sqr(dis/epsilon[i]))/(Foam::pow(epsilon[i],3)*Foam::pow(Foam::constant::mathematical::pi,1.5)));
                            thrustBodyForceSum += (-bladeForce[i][j][k] * (Foam::exp(-Foam::sqr(dis/epsilon[i]))/(Foam::pow(epsilon[i],3)*Foam::pow(Foam::constant::mathematical::pi,1.5))) *
                                                    mesh.V()[sphereCells[i][m]]) & uvShaft[i];
                            torqueBodyForceSum += ( bladeForce[i][j][k] * (Foam::exp(-Foam::sqr(dis/epsilon[i]))/(Foam::pow(epsilon[i],3)*Foam::pow(Foam::constant::mathematical::pi,1.5))) * 
                                                    bladeRadius[i][j][k] * Foam::cos(PreCone[n][j]) * mesh.V()[sphereCells[i][m]]) & bladeAlignedVectors[i][j][1];
                        }
                    }
                }  
            }
        }
        thrustSum += thrust[i];//推力加和整个水轮机阵列
        torqueSum += torqueRotor[i];//扭矩加和整个水轮机阵列
    }
    reduce(thrustBodyForceSum,sumOp<scalar>());
    reduce(torqueBodyForceSum,sumOp<scalar>());

    // Print information comparing the actual thrust and torque to the integrated body force.
    Info << "Thrust from Body Force = " << thrustBodyForceSum << tab << "Thrust from Act. Line = " << thrustSum << tab << "Ratio = " << thrustBodyForceSum/thrustSum << endl;
    Info << "Torque from Body Force = " << torqueBodyForceSum << tab << "Torque from Act. Line = " << torqueSum << tab << "Ratio = " << torqueBodyForceSum/torqueSum << endl;


if(Vflow != 0)
{
    // Perform rotation turbine by turbine.
    forAll(uvShaft, i)
    {	

	forAll(bladePoints[i], j)//对所有叶片坐标点进行拖曳
        {
            forAll(bladePoints[i][j], k)
            {
                bladePoints[i][j][k].x() -= Vflow*dt; 
            }
        }   

        rotorApex[i].x() -= Vflow*dt;
    }



    sphereCells.clear();//从for循环中移出，防止在循环过程中清除数组前序数据
    for(int i = 0; i < numTurbines; i++)
    {
       // First compute the radius of the force projection (to the radius
        // where the projection is only 0.001 its maximum value - this seems
        // recover 99.9% of the total forces when integrated).
        projectionRadius.append(epsilon[i] * Foam::sqrt(Foam::log(1.0/0.001)));

        // Calculate the sphere of influence radius.
        scalar sphereRadius = 0.0;//定义影响半径，在此半径内的网格才用于水轮机的计算
        int j = turbineTypeID[i];
        forAll(PreCone[j],k)
        {
            scalar sphereRadiusI = Foam::sqrt(Foam::sqr((OverHang[j] + UndSling[j]) + TipRad[j]*Foam::sin(PreCone[j][k])) + Foam::sqr(TipRad[j]*Foam::cos(PreCone[j][k])));
            if(sphereRadiusI > sphereRadius)
            {
                sphereRadius = sphereRadiusI;
            }
        } 
        sphereRadius += projectionRadius[i];

        // Find the cells within the sphere of influence.
        DynamicList<label> sphereCellsI;
        towerShaftIntersect[i].x() -= Vflow*dt;
        sphereCellsI.clear();
        // Find the cells within the sphere of influence.
        forAll(mesh.cells(),cellI)
        {
            if (mag(mesh.C()[cellI] - towerShaftIntersect[i]) <= sphereRadius)
            {
                sphereCellsI.append(cellI);
            }
        }
        sphereCells.append(sphereCellsI);
        sphereCellsI.clear();

        // Create a list of turbines that this processor could forseeably control.
        // If sphereCells[i] is not empty, then turbine i belongs in the list.
        if (sphereCells[i].size() > 0)
        {
            turbinesControlled.append(i);
        }
    }
}



    // Print turbine output to file.
        outputIndex++;

        if (outputControl == "timeStep")
        {
            if (outputIndex >= outputInterval)
    	    {
	        outputIndex = 0;
    if (Pstream::master())
    {


        forAll(bladePoints,i)
        {
            // Write out time and delta t.
            *torqueRotorFile_ << i << " " << time << " " << dt << " ";
            *torqueGenFile_ << i << " " << time << " " << dt << " ";
            *thrustFile_ << i << " " << time << " " << dt << " ";
            *powerRotorFile_ << i << " " << time << " " << dt << " ";
            *rotSpeedFile_ << i << " " << time << " " << dt << " ";
            *rotSpeedFFile_ << i << " " << time << " " << dt << " ";
            *azimuthFile_ << i << " " << time << " " << dt << " ";
            *pitchFile_ << i << " " << time << " " << dt << " ";
            *nacYawFile_ << i << " " << time << " " << dt << " ";

            // Write out information for each turbine.
            *torqueRotorFile_ << torqueRotor[i]*fluidDensity[i] << endl;
            *torqueGenFile_ << torqueGen[i] << endl;
            *thrustFile_ << thrust[i]*fluidDensity[i] << endl;
            *powerRotorFile_ << powerRotor[i]*fluidDensity[i] << endl;
            *rotSpeedFile_ << rotSpeed[i]/rpmRadSec << endl;
            *rotSpeedFFile_ << rotSpeedF[i]/rpmRadSec << endl;
            *azimuthFile_ << azimuth[i]/degRad << endl;
            *pitchFile_ << pitch[i] << endl;
            *nacYawFile_ << standardToCompass(nacYaw[i]/degRad) << endl;

            // Proceed blade by blade.
            forAll(bladePoints[i], j)
            {
                // Write out time and delta t.
                *alphaFile_ << i << " " << j << " " << time << " " << dt << " ";
                *VmagFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *VaxialFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *VtangentialFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *VradialFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *ClFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *CdFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *liftFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *dragFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *axialForceFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *tangentialForceFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *xFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *yFile_ << i << " " << j << " " <<  time << " " << dt << " ";
                *zFile_ << i << " " << j << " " <<  time << " " << dt << " ";

                forAll(bladePoints[i][j], k)
                {   
                    *alphaFile_ << alpha[i][j][k] << " ";
                    *VmagFile_ << Vmag[i][j][k] << " ";
                    *VaxialFile_ << windVectors[i][j][k].x() << " ";
                    *VtangentialFile_ << windVectors[i][j][k].y() << " ";
                    *VradialFile_ << windVectors[i][j][k].z() << " ";
                    *ClFile_ << Cl[i][j][k] << " ";
                    *CdFile_ << Cd[i][j][k] << " ";
                    *liftFile_ << lift[i][j][k]*fluidDensity[i] << " ";
                    *dragFile_ << drag[i][j][k]*fluidDensity[i] << " ";
                    *axialForceFile_ << axialForce[i][j][k]*fluidDensity[i] << " ";
                    *tangentialForceFile_ << tangentialForce[i][j][k]*fluidDensity[i] << " ";
                    *xFile_ << bladePoints[i][j][k].x() << " ";
                    *yFile_ << bladePoints[i][j][k].y() << " ";
                    *zFile_ << bladePoints[i][j][k].z() << " ";
                }
                *alphaFile_ << endl;
                *VmagFile_ << endl;
                *VaxialFile_ << endl;
                *VtangentialFile_ << endl;
                *VradialFile_ << endl;
                *ClFile_ << endl;
                *CdFile_ << endl;
                *liftFile_ << endl;
                *dragFile_ << endl;
                *axialForceFile_ << endl;
                *tangentialForceFile_ << endl;
                *xFile_ << endl;
                *yFile_ << endl;
                *zFile_ << endl;
            }
        }
          
        *torqueRotorFile_ << endl;
        *torqueGenFile_ << endl;
        *thrustFile_ << endl;
        *powerRotorFile_ << endl;
        *rotSpeedFile_ << endl;
        *rotSpeedFFile_ << endl;
        *azimuthFile_ << endl;
        *pitchFile_ << endl;
        *nacYawFile_ << endl;

        *alphaFile_ << endl;
        *VmagFile_ << endl;
        *VaxialFile_ << endl;
        *VtangentialFile_ << endl;
        *VradialFile_ << endl;
        *ClFile_ << endl;
        *CdFile_ << endl;
        *liftFile_ << endl;
        *dragFile_ << endl;
        *axialForceFile_ << endl;
        *tangentialForceFile_ << endl;
        *xFile_ << endl;
        *yFile_ << endl;
        *zFile_ << endl;
    }
	    }
        }
/*        else if (outputControl == "runTime")
        {
            if ((runTime_.value() - lastOutputTime) >= outputInterval)
            {
    	        lastOutputTime += outputInterval;
	        printOutputFiles();
            }
        }
        else
        {
            printOutputFiles();
        }
*/
    // Now that at least the first time step is finished, set pastFirstTimeStep
    // to true.
    pastFirstTimeStep = true;
