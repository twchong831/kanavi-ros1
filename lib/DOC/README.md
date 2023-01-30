# Carnavicom co.,ltd. LiDAR LIB

## Development ENV.

| **Support**    | **OS**  |
| ------------------|---|
| ✅   |Ubuntu 20.04    |
| ✅   |mac OS          |


### Support LiDAR Device

|      | ***Name***    | ***Desciption***                          |
| ---- | --------------------- | ------------------------------------------------- |
| ✅   | VL-AS16               | 16ch 145$\text{\textdegree}$ LiDAR Sensor       |
| ⬜️ | VL-L001IK01           | 1ch laser range finder                            |
| ⬜️ | VL-R001IK01           | 1ch 120$\text{\textdegree}$ LiDAR Sensor        |
| ✅   | **VL-R002IF01** | 2ch 120$\text{\textdegree}$ LiDAR Sensor (R2)   |
| ✅   | **VL-R001IK02** | 1ch 300$\text{\textdegree}$ LiDAR Sensor (R300) |
| ⬜️ | VL-R004IK01           | 4ch 90$\text{\textdegree}$ LiDAR Sensor?        |
| ⬜️ | VL-R004IK02           | 4ch 90$\text{\textdegree}$ LiDAR Sensor??       |

## How to build Carnavicom LiDAR LIB

```powershell
make all 	# install carnavicom LIB
```

### where LIB

```powershell
/usr/local/lib
/usr/local/include/CARNAVICOM_LIB
```

### Hot to Remove

```powershell
make remove
```

### visualization Example

```powershell
./example/src
```

- Displaying LiDAR data in top view using Carnavicom LIB and opencv.

## Update List

- 2022.06.01 First Generated
- 2022.07.05 add LICENSE
- 2023.01.25 support MAC os