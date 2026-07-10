# Coaxial 3-RRR Spherical Parallel Manipulator (SPM)

## control system for coaxial 3-rrr spherical parallel manipulator on teensy 4.1 and esp32. includes simulation with gazebo and digital twin via ros2

### design of current iteration (v2) and matlab simulation of previous iteration (v1)
<div align="center">
  <table>
    <tr>
      <!-- The tall image on the left spans all 3 rows -->
      <td rowspan="3">
        <img src="https://github.com/user-attachments/assets/d7fb10b2-331c-4ca2-a907-060c54faf143" width="550" alt="Tall Main Picture" />
      </td>
      <!-- Row 1 right side -->
      <td>
        <img src="https://github.com/user-attachments/assets/9539d420-d672-46e3-af1d-bf7350fe3b23?raw=true" width="225" alt="Small GIF 1" />
      </td>
    </tr>
    <tr>
      <!-- Row 2 right side -->
      <td>
        <img src="https://github.com/user-attachments/assets/554e3f9e-d0cf-429b-919d-dce808206e2c?raw=true" width="225" alt="Small GIF 2" />
      </td>
    </tr>
    <tr>
      <!-- Row 3 right side (Added missing tr tag wrapper) -->
      <td>
        <img src="https://github.com/user-attachments/assets/7e3b4d92-11b7-4cc3-826d-0ba2fa172675?raw=true" width="225" alt="Small GIF 3" />
      </td>
    </tr>
  </table>
</div>
<br>

### inverse kinematics trajectory generation (3rd order polynomial) + verification (forward kinematics using newton-raphson) of current iteration; error on order of e-13. based on Cruz-Reyes et al.
<div align="center">
  <img width="1000" alt="image" src="https://github.com/user-attachments/assets/87804548-89f3-4ffc-aa93-ebdca8287da7" />
</div>
