import numpy as np
from scipy.interpolate import CubicSpline
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))


dis = 1
angle = 90
deltaAngle = angle / 3
scale = 0.1

pathStartAll = []
pathAll = []
pathList = []
pathID = 0
groupID = 0

# Ackermann limits
wheelbase = 0.3
deltaMax = 30 * np.pi / 180
kappaMax = np.tan(deltaMax) / wheelbase

theta0 = 0.0
maxYawChange = np.pi / 2
ds = 0.05


# utility functions
def compute_curvature(x, y):
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    kappa = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    kappa[np.isnan(kappa)] = 0
    return kappa


def spline(x, y, xq):
    """1D spline interpolation"""
    cs = CubicSpline(x, y, extrapolate=True)
    return cs(xq)


# generating paths
print("\nGenerating paths")

for shift1 in np.arange(-angle, angle + 1e-6, deltaAngle):
    wayptsStart = np.array([[0, 0, 0],
                            [dis, shift1, 0]])

    pathStartR = np.arange(0, dis + 1e-6, 0.01)
    pathStartShift = spline(wayptsStart[:, 0], wayptsStart[:, 1], pathStartR)

    pathStartX = pathStartR * np.cos(pathStartShift * np.pi / 180)
    pathStartY = pathStartR * np.sin(pathStartShift * np.pi / 180)
    pathStartZ = np.zeros_like(pathStartX)

    pathStart = np.vstack([pathStartX, pathStartY, pathStartZ,
                            np.ones_like(pathStartX) * groupID])
    pathStartAll.append(pathStart)

    for shift2 in np.arange(-angle * scale + shift1,
                            angle * scale + shift1 + 1e-6,
                            deltaAngle * scale):
        for shift3 in np.arange(-angle * scale**2 + shift2,
                                angle * scale**2 + shift2 + 1e-6,
                                deltaAngle * scale**2):

            waypts = np.array([
                [pathStartR[-1], pathStartShift[-1], 0],
                [2 * dis, shift2, 0],
                [3 * dis - 0.001, shift3, 0],
                [3 * dis, shift3, 0]
            ])

            pathR = np.arange(0, waypts[-1, 0] + 1e-6, ds)

            # frenet lateral offset spline
            latOffset = spline(waypts[:, 0], waypts[:, 1],
                               np.minimum(pathR, waypts[-1, 0]))
            dLat = np.gradient(latOffset, pathR)
            ddLat = np.gradient(dLat, pathR)

            # enforce curvature limits
            scaleFactor = 1.0
            for _ in range(12):
                kappaTest = np.abs(scaleFactor * ddLat /
                                   (1 + (scaleFactor * dLat)**2)**1.5)
                if np.max(kappaTest) <= kappaMax * 1.0001:
                    break
                scaleFactor *= 0.75

            latOffset *= scaleFactor
            dLat *= scaleFactor
            ddLat *= scaleFactor

            theta = np.unwrap(np.arctan(dLat))
            kappa = ddLat / (1 + dLat**2)**1.5

            # prevent backwards motion
            pathX = np.zeros_like(theta)
            pathY = np.zeros_like(theta)
            pathY[0] = latOffset[0]

            for k in range(1, len(theta)):
                dx = ds * np.cos(theta[k - 1])
                if dx <= 0:
                    dx = np.finfo(float).eps
                pathX[k] = pathX[k - 1] + dx
                pathY[k] = pathY[k - 1] + ds * np.sin(theta[k - 1])

            # U-turn handling
            if pathX[-1] < 0:
                extendFactorMax = 4
                for ef in range(2, extendFactorMax + 1):
                    newMaxS = ef * waypts[-1, 0]
                    pathR_new = np.arange(0, newMaxS + 1e-6, ds)
                    latOffset_new = spline(waypts[:, 0], waypts[:, 1],
                                            np.minimum(pathR_new, waypts[-1, 0]))
                    dLat_new = np.gradient(latOffset_new, pathR_new)
                    ddLat_new = np.gradient(dLat_new, pathR_new)
                    theta_new = np.unwrap(np.arctan(dLat_new))

                    pathX_new = np.zeros_like(theta_new)
                    pathY_new = np.zeros_like(theta_new)
                    pathY_new[0] = latOffset_new[0]

                    for k2 in range(1, len(theta_new)):
                        dx = ds * np.cos(theta_new[k2 - 1])
                        if dx <= 0:
                            dx = np.finfo(float).eps
                        pathX_new[k2] = pathX_new[k2 - 1] + dx
                        pathY_new[k2] = pathY_new[k2 - 1] + ds * np.sin(theta_new[k2 - 1])

                    pathAngle = np.arctan2(pathY_new[-1], pathX_new[-1])
                    angleDiff = (pathAngle - theta0 + np.pi) % (2 * np.pi) - np.pi
                    kappa_new = ddLat_new / (1 + dLat_new**2)**1.5

                    if (np.max(np.abs(kappa_new)) <= kappaMax * 1.0001 and
                        pathX_new[-1] >= 0 and
                        abs(angleDiff) <= maxYawChange):
                        pathX, pathY, theta, kappa = pathX_new, pathY_new, theta_new, kappa_new
                        break

            # final curvature check
            if np.max(np.abs(kappa)) > kappaMax:
                continue

            # save path
            pathZ = np.zeros_like(pathX)
            path = np.vstack([pathX, pathY, pathZ,
                              np.ones_like(pathX) * pathID,
                              np.ones_like(pathX) * groupID])
            pathAll.append(path)
            pathList.append([pathX[-1], pathY[-1], pathZ[-1], pathID, groupID])
            pathID += 1

    groupID += 1
    print("groupID =", groupID)

print("Total paths:", pathID)


# save paths as PLY
def save_ply(filename, data, properties):
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {data.shape[1]}\n")
        for p in properties:
            f.write(p + "\n")
        f.write("end_header\n")
        fmt = " ".join(["%f"] * (data.shape[0] - 2) + ["%d", "%d"])
        np.savetxt(f, data.T, fmt=fmt)


pathStartAll_arr = np.hstack(pathStartAll)
pathAll_arr = np.hstack(pathAll)
pathList_arr = np.array(pathList).T

save_ply("startPaths.ply", pathStartAll_arr,
         ["property float x",
          "property float y",
          "property float z",
          "property int group_id"])

save_ply("paths.ply", pathAll_arr,
         ["property float x",
          "property float y",
          "property float z",
          "property int path_id",
          "property int group_id"])

save_ply("pathList.ply", pathList_arr,
         ["property float end_x",
          "property float end_y",
          "property float end_z",
          "property int path_id",
          "property int group_id"])

print("\nPLY files written.")