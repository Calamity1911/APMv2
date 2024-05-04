<h1>Automotive Performance Monitor Rev. 2</h1>

<p>
  The Automotive Performance Monitor was a college senior design project codeveloped by four undergrad students from the Milwaukee School of Engineering. Those original students were Bayan Al Rebh, Hunter Fritchen, Evan Heinrich (repo owner, Calamity1911), and Brayden Mleziva (repo contributor, Beboper12).
</p>
<p>
  The Automotive Performance Monitor project was intended to pe an open-source powerful automotive diagnostic tool, with a few features added on to satisfy senior design requirements. This version is a fork of the original project code, which is attached to the repo as the file "legacy_apm.zip"
</p>
<p>
  The target platform is Espressif's ESP32. For the original project, we had used the ESP32-S3 N8R2 variant. The original was also only designed for use on CAN bus enabled systems, which for now is a carryover until/unless Brayden intends to add support for his vehicle's GM proprietary interface. The implications of this are that as it currently stands, this project will only be guarunteed to work on USDM vehicles manufactured after the 2008 model year, which is when manufacturers were first required to expose OBD-II functionality on a CAN bus. 
</p>
<p>
  As this is now a personal project, I (Evan) am now scrapping a bunch of the work from the original APM project, as it was written with time constraints, and as such, a good chunk of the code is not really optimal. I will be reusing certain aspects of the original code, though. My (Evan) intents with this fork was to clean up the OBD-II library which would then make it easier to implement a user interface for. I also stripped the features I do not personally need, so this project will only contain support for OBD-II data readouts via PIDs (Parameter Identifiers) and Diagnostic Trouble Code reading and clearing.
</p>
<p>My (Evan) final goal with this fork was to clean up the backend of the user interface code. The user interface was designed as a captive wireless access point hosting a HTTP server. Think those public WiFi network where your phone prompts you to sign in. In our case though, instead of signing in, it just pulls up the main user interface. Anyway, the original APM utilized WebSockets to enable live data readouts, but I (Evan) felt like they weren't utilized to their full potential, again due to time restraints.</p>

<h2>For more information on what is currently being worked on, see the "TODO.txt" file in the repository.</h2>
