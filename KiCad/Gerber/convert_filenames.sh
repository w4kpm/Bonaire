#! /bin/bash
mv Helios-B.Cu.gbr Helios.GBL
mv Helios-F.Cu.gbr Helios.GTL
mv Helios-F.Mask.gbr Helios.GTS
mv Helios-B.Mask.gbr Helios.GBS
mv Helios-F.SilkS.gbr Helios.GTO
mv Helios-B.SilkS.gbr Helios.GBO
mv Helios-Edge.Cuts.gbr Helios.GML
mv Helios.drl Helios.TXT
zip HeliosCompiled.zip Helios.*
