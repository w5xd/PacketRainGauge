# Packet Rain Gauge


<b>As of this writing, this project is NOT complete. Do NOT try to duplicate this yet.</b>

This design is for a PCB that telemeters a packet when a magnet approaches, and
again when it retreats from a hall effect sensor (Si7210) on the PCB. It also includes a 3D
printable PCB enclosure to retrofit inside a discarded rain gauge funnel
from an Oregon Scientific RGR126N Wireless Rain Gauge. If you don't 
have an RGR126N to retrofit, its left as an exercise
for the reader to design a funnel, rocker, and weather-tight
battery compartment.

This unit telemeters to 
<a href='https://github.com/w5xd/PacketGateway'>an open-source packet gateway</a>. 

The rain gauge mechanical design is simple. A funnel directs rainfall into a rocker
that is centered below the funnel. The rocker has two identical cups arranged on a see-saw. After about 1mm
of rainfail, the higher cup has enough weight to rock the see-saw. The higher
one falls to become the lower one and dumps its water from the now lowered cup. The new position leaves 
the opposite
cup&mdash;which was empty&mdash;under the funnel. It begins to fill with new rainfall and the process repeats. In the middle of the
rocker&mdash;centered so it does not affect the center of mass between the two cups&mdash;is a magnet. 
The PCB detects the magnet swinging
by. 

<p align='center'><img src='RGR126Nfunnel.jpg' alt='RGR126Nfunnel.jpg'/></p>

The RGR126N design sensed the passing of the magnet using a reed relay. Such a relay has ferro magnetic 
contacts that close as
the magnet is in proximity. The relay was positioned for closest approach 
as the rocker passed top dead center
in both directions, so the rocker was moving relatively quickly.  

The
Si7210 sensor used in this design senses at 5 times per second which is a little slow to
reliably detect fast passage of the magnet through top dead center. However, the Si7210 can
easily be programmed to separately detect the arrival and the departure of the magnet from
its proximity. This design positions its sensor in alignment with the rocker at its resting position with one
cup down&mdash;the left one in the photo above. Its Arduino
sketch is programmed to separately report the arrival and departure of the rocker, sending a packet for
each. The new magnet is mounted in the same position as the old one. Its not quite the same mass: about 0.5g as
opposed to 0.3g on my scale, so a calibration check is in order before putting it into service.
The result is this design sends packets at the same time as the old one. 

A K&J Magnetics B422 magnet matches the original magnet's dimensions.
 The original magnet cannot be retained because the reed relay required the magnet's
pole axis orientation be parallel to the axis of the reed relay. That pole
orientation is not appropriate for the hall effect sensor on this
replacement PCB. For the Si7210 sensor to best sense proximity, the pole axis should penetrate the sensor.
The sensor manufacturer publishes this discussion about how the
magnet and sensor geometry are designed: 
<a href='https://www.silabs.com/documents/public/application-notes/an1018-si72xx-sensors.pdf'>
https://www.silabs.com/documents/public/application-notes/an1018-si72xx-sensors.pdf<a>

The original
battery compartment is retained after removing the PCB that shares the
battery enclosure.

This retrofit parts list is:
<ul>
<li> The original funnel's plastic parts minus the two PCBs, and with the
original magnet removed.
<li> A PCB that mounts the original relay along with an Arduino, 
a TMP175 sensor, and an RFM69 packet radio module. Gerber files are <a href='./PCB'>here</a>.
<li> A <a href='https://www.kjmagnetics.com/proddetail.asp?prod=B422'>K&J Magnetics B422 neodymium magnet</a>. Its
dimensions are 1/8" x 1/8" x 1/4" which fit into the same mounting hole on the original rocker.
Note there is a required orientation of the new magnet that takes some care. See <a href='#MAGNET_ORIENTATION'>below</a>.
<li> A 3D printable enclosure that mounts on the same hole pattern
as the original PCB facing the magnet on the rocker.
<li> Two AA cells inside the original plastic battery compartment. The
battery's original +3VDC and GND wires are replaced with
a pair of wires to the replacement PCB. This design can use either
alkaline or lithium cells. The latter give somewhat longer
life, and operate to lower temperatures (although there won't be any rain down there
way below 0 degrees F where the lithium still works and the alkaline does not.)
</ul>

<h3>The Arduino sketch</h3>
The sketch sends a packet to the Packet Gateway every time the 
magnet on the rocker arm arrives at, or departs from the
sensor. The message
packet also contains a battery voltage measurement, the TMP175
reading, and other details. The sketch also sends a packet after
a timed interval absent any rainfail, which enables battery level
monitoring at the gateway. The timed interval is settable, and
once per 24 hours is recommended. The temperature reading at the funnel is 
useful mainly for
monitoring the condition of the funnel assembly. Direct
sunlight on the funnel makes its value not representative
of ambient air temperature. But the funnel must be 
mounted in the open to be useful as a rain gauge.

The gateway's processing of the rainfall messages is not part of this
repository. See the repository at https://github.com/w5xd/diysha for
an example. In that project, the receipt of rainfall packets writes a
text file that, in turn, can be read by the "bins" feature in <a href='http://www.gnuplot.info'>gnuplot</a>.

<h2> Construction</h2>

The <a href='https://www.sparkfun.com/products/11114'>Arduino Pro Mini</a> requires its 
following PCB options to be made in order to work 
in this project:
<ul>
<li>The 3.3V version of the Pro Mini is <b>required</b>.
<li>Jumper SJ1 (top side, close to the GND pad) must be desoldered to remove the red power LED's power drain.
<li>The bottom side pullup positions, R1 and R3, must each have a 4.7K 
resistor installed. Two SMD 0603 size resistors are required and just fit inside a hole
in the PCB designed to clear them.
<li>The 330 ohm resistor just inside pins D11 and D12 must be removed (or cut with a diagonal 
cutter.) This
disables the green LED to prevent its battery drain and load the the SCK line.
</ul>

<h3>PCB considerations</h3>
Mount the Arduino directly to the PCB without headers. Its important to
<ul>
<li>center the Arduino on its holes in order to clear the I2C resistors mounted as above. 
<li>minimize the solder bumps on the bottom side of the PCB so the assembly will later
fit in the enclosure. I set the PCB on a flat surface, laid the Arduino board on top,
then inserted headers from the top and soldered from the top first, so the pins just
reach the bottom of the main PCB.
</ul>

The replacement PCB layout is optimized to be as small as possible while 
retaining the ability for an ordinary human to place the parts and solder them.
I used a home brew SMD oven, but steady hands can alternatively 
solder them one pin at a time.

Its circuit diagram is <a href='PCB-circuit.pdf'>here</a>.

The Si7210 is mounted to the <i>bottom</i> of the PCB. This is the only part on the bottom.
I used an SMD over to mount it (and nothing else with it.) Then flipped the board over
and used the oven again to mount all the top side SMD parts. The RFM69 is supposed to be
oven-safe, but I have destroyed at least one (maybe not because of the oven?) and its
easy enough to hand solder its 100 thou solder pads.

Setting up the Arduino requires programming the part, and also requires 
serial port commands to configure the radio parameters. The permanent "connector" on
the board for its serial port requires a pogo adapter to access. The standard
programming header along the shorter side of the Arduino also works once, but
you'll have to cut off all the pins to mount it in the enclosure.

The permanent serial connector is based on the 6-pin ISP header layout on many
Arduino boards, but this board has 9 pins. At least one 
<a href=''>SparkFun ISP Pogo Adapter</a>
can be used to setup this PCB. 

If you position the Pogo Adapter on the 6 pins inside the yellow silkscreen rectangle,
it sees the standard 6-pin ISP programming connectors. But position it
on the 6 pins aligned along the nearest edge of the PCB, and it connects to the Arduino's
serial port pins. This pin-out is not standard, but the Pogo adapter as supplied
has no connector on the ends of its wires. You can solder them to, for example,
the <a href='https://www.sparkfun.com/products/13263'>SparkFun FT231X Breakout</a>.
Note that on this particular FT231X breakout, you <b>must</b> change the power jumper to
make it supply 3.3V, else you'll fry the RFM69 on your first try.

The Pogo Adapter has 6 wires. When positioned for access to the Arduino serial port,
the wires are <i>not</i> as labeled on the Pogo board, but are as follows on the PCB's
Arduino:
<ol>
<li>black. 3.3VDC. <i>Not</i> 5V, which will destroy the RFM69.
<li>red. MOSI. Leave open.
<li>orange. GND
<li>yellow.  RXI on Arduino. To TXD on break out.
<li>green. TXO on Arduino. Goes to RXI on break out.
<li>blue. DTR on Arduino. DTR on the break out.
<li>To match the Arduino-to-Serial-breakout standard wiring, you also must put a
jumper directly on the break out between its ground and CTS.
</ol>

A Pogo adapter wired as above to a serial break out can be used both to
program the Arduino, and to use a terminal application for serial port
commands to configure the packet radio parameters.

A Pogo adapter can also be used posititioned to its standard wire assignments
to program the Arduino, but the radio parameters
can only be configured through the serial port.

<h3>Enclosure</h3>
The enclosure prints as two parts. 
The base has three mounting holes matching the original rocker-mounted PCB. 
And it has three holes
for wires: one each for the radio antenna, ground, and 3.3V. 
The top slides over the base. Once the device is configured, use a silicon sealant
on the joints between the base and cover, and also to seal the wire holes.
Consider the enclosure as disposable. If you need to access the PCB after 
sealing it, plan to destroy the enclosure and print a new one.

<h3 id="MAGNET_ORIENTATION">Magnet Orientation</h3>
The B422 magnet's 1/4" dimension fits into the rocker mount in the obvious 1/4" slot dimension. But there are
four 1/8" sides to the magnet that might face up! The magnet has two opposing 1/8" faces that work when facing 
the sensor
 (the ones with the poles) and the other two 1/8" faces are no good! It requires another
magnet with known poles to figure out which face is right. Here
are some: <a href='https://www.kjmagnetics.com/products.asp?cat=163'>
https://www.kjmagnetics.com/products.asp?cat=163</a>.  The B422 may be mounted with either
its North pole or South pole facing the PCB. (One orientation will permanently give positive
readings in the magnetic sensor, the other will give negative.)