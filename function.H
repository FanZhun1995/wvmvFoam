vector rotatePoint(vector point, vector rotationPoint, vector axis, scalar angle)//定义旋转函数，旋转点（对象点，）
{
    // Declare and define the rotation matrix.
    tensor RM;
    RM.xx() = Foam::sqr(axis.x()) + (1.0 - Foam::sqr(axis.x())) * Foam::cos(angle); 
    RM.xy() = axis.x() * axis.y() * (1.0 - Foam::cos(angle)) - axis.z() * Foam::sin(angle); 
    RM.xz() = axis.x() * axis.z() * (1.0 - Foam::cos(angle)) + axis.y() * Foam::sin(angle);
    RM.yx() = axis.x() * axis.y() * (1.0 - Foam::cos(angle)) + axis.z() * Foam::sin(angle); 
    RM.yy() = Foam::sqr(axis.y()) + (1.0 - Foam::sqr(axis.y())) * Foam::cos(angle);
    RM.yz() = axis.y() * axis.z() * (1.0 - Foam::cos(angle)) - axis.x() * Foam::sin(angle);
    RM.zx() = axis.x() * axis.z() * (1.0 - Foam::cos(angle)) - axis.y() * Foam::sin(angle);
    RM.zy() = axis.y() * axis.z() * (1.0 - Foam::cos(angle)) + axis.x() * Foam::sin(angle);
    RM.zz() = Foam::sqr(axis.z()) + (1.0 - Foam::sqr(axis.z())) * Foam::cos(angle);

    // Rotation matrices make a rotation about the origin, so need to subtract rotation point
    // off the point to be rotated.
    point = point - rotationPoint;

    // Perform the rotation.
    point = RM & point;

    // Return the rotated point to its new location relative to the rotation point.
    point = point + rotationPoint;

    return point;
}


scalar interpolate(scalar xNew, DynamicList<scalar>& xOld, DynamicList<scalar>& yOld)
{
    label index = 0;
    label indexP = 0;
    label indexM = 0;
    scalar error = 1.0E30;
    forAll(xOld, i)
    {
        scalar diff = mag(xNew - xOld[i]);
        if(diff < error)
        {
            index = i;
            error = diff;
        }
    }
    if (xNew < xOld[index])
    {
        if (index == 0)
        {
            indexP = 1;
            indexM = indexP - 1;
        }
        else
        {
            indexP = index;
            indexM = indexP - 1;
        }
        return yOld[indexM] + ((yOld[indexP] - yOld[indexM])/(xOld[indexP] - xOld[indexM]))*(xNew - xOld[indexM]);
    }
    else if (xNew > xOld[index])
    {
        if (index == xOld.size() - 1)
        {
            indexP = xOld.size() - 1;
            indexM = indexP - 1;
        }
        else
        {
            indexP = index + 1;
            indexM = indexP - 1;
        }
        return yOld[indexM] + ((yOld[indexP] - yOld[indexM])/(xOld[indexP] - xOld[indexM]))*(xNew - xOld[indexM]);
    }
    else if (xNew == xOld[index])
    {
        return yOld[index];
    }
    else
    {
        return 0.0;
    }
}


label interpolate(scalar xNew, DynamicList<scalar>& xOld, DynamicList<label>& yOld)
{
    label index = 0;
    label indexP = 0;
    label indexM = 0;
    scalar error = 1.0E30;
    forAll(xOld, i)
    {
        scalar diff = mag(xNew - xOld[i]);
        if(diff < error)
        {
            index = i;
            error = diff;
        }
    }
    if (xNew < xOld[index])
    {
        if (index == 0)
        {
            indexP = 1;
            indexM = indexP - 1;
        }
        else
        {
            indexP = index;
            indexM = indexP - 1;
        }
        return round(yOld[indexM] + ((yOld[indexP] - yOld[indexM])/(xOld[indexP] - xOld[indexM]))*(xNew - xOld[indexM]));
    }
    else if (xNew > xOld[index])
    {
        if (index == xOld.size() - 1)
        {
            indexP = xOld.size() - 1;
            indexM = indexP - 1;
        }
        else
        {
            indexP = index + 1;
            indexM = indexP - 1;
        }
        return round(yOld[indexM] + ((yOld[indexP] - yOld[indexM])/(xOld[indexP] - xOld[indexM]))*(xNew - xOld[indexM]));
    }
    else if (xNew == xOld[index])
    {
        return yOld[index];
    }
    else
    {
        return 0.0;
    }
}

scalar compassToStandard(scalar dir)
{
    dir += 180.0;
    if (dir >= 360.0)
    {
       dir -= 360.0;
    }
    dir = 90.0 - dir;
    if (dir < 0.0)
    {
        dir = dir + 360.0;
    }
    return dir;
}

scalar standardToCompass(scalar dir)
{
    dir = 90.0 - dir;
    if (dir < 0.0)
    {
        dir += 360.0;
    }
    dir += 180.0;
    if (dir >= 360.0)
    {
        dir -= 360.0;
    }
    return dir;
}
