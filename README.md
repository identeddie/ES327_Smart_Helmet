# ES327 - Smart Helmet

Documentation for use of code:
- Make sure to modify boxed out "Setup Area" when uploading to nodes
- Every node needs a unique ID number, an ID of 0 is normally reserved for admin
- The setup signal can be changed and should be interpreted as a string of bytes (in hex)

The MSB of every byte is the leadStatus indicator. A leader of a team should have this set.
The remaining bits in the byte indicate the team number
Each hex value (two characters in the string) relates to an individual node starting with node 0
Example: The leader of team 2 would need to be assigned 1000_0010, or 0x82 -> "82" in the setup string

- Threshold values for the sensors can similarly be tailored for the appropriate scenario
