<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="22308000">
	<Property Name="NI.LV.All.SourceOnly" Type="Bool">true</Property>
	<Item Name="My Computer" Type="My Computer">
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="allInOne.vi" Type="VI" URL="../allInOne.vi"/>
		<Item Name="fuzzyProject.vi" Type="VI" URL="../fuzzyProject.vi"/>
		<Item Name="pidProject.vi" Type="VI" URL="../pidProject.vi"/>
		<Item Name="STFPID.vi" Type="VI" URL="../STFPID.vi"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="Clear Errors.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/error.llb/Clear Errors.vi"/>
				<Item Name="NI_AALBase.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AALBase.lvlib"/>
				<Item Name="NI_AALPro.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AALPro.lvlib"/>
				<Item Name="NI_Fuzzy_Logic_API.lvlib" Type="Library" URL="/&lt;vilib&gt;/addons/Fuzzy Logic/NI_Fuzzy_Logic_API.lvlib"/>
				<Item Name="NI_Gmath.lvlib" Type="Library" URL="/&lt;vilib&gt;/gmath/NI_Gmath.lvlib"/>
				<Item Name="NILVSim.dll" Type="Document" URL="/&lt;vilib&gt;/Simulation/Implementation/Shared/NILVSim.dll"/>
				<Item Name="SIM limit type.ctl" Type="VI" URL="/&lt;vilib&gt;/Simulation/Implementation/Shared/SIM limit type.ctl"/>
				<Item Name="SIM MIMO State Space.ctl" Type="VI" URL="/&lt;vilib&gt;/Simulation/Shared/_Datatypes/SIM MIMO State Space.ctl"/>
				<Item Name="SIM PID (Academic).vi" Type="VI" URL="/&lt;vilib&gt;/Simulation/Model Libraries/PID/SIM PID (Academic).vi"/>
				<Item Name="SIM PID (Parallel).vi" Type="VI" URL="/&lt;vilib&gt;/Simulation/Model Libraries/PID/SIM PID (Parallel).vi"/>
				<Item Name="SIM PID (Series).vi" Type="VI" URL="/&lt;vilib&gt;/Simulation/Model Libraries/PID/SIM PID (Series).vi"/>
				<Item Name="SIM PID Derivative Gain.vi" Type="VI" URL="/&lt;vilib&gt;/Simulation/Model Libraries/PID/subVIs/SIM PID Derivative Gain.vi"/>
				<Item Name="SIM PID Derivative Series.vi" Type="VI" URL="/&lt;vilib&gt;/Simulation/Model Libraries/PID/subVIs/SIM PID Derivative Series.vi"/>
				<Item Name="SIM PID Integral Gain.vi" Type="VI" URL="/&lt;vilib&gt;/Simulation/Model Libraries/PID/subVIs/SIM PID Integral Gain.vi"/>
				<Item Name="SIM PID.vi" Type="VI" URL="/&lt;vilib&gt;/Simulation/Model Libraries/PID/SIM PID.vi"/>
			</Item>
			<Item Name="lvanlys.dll" Type="Document" URL="/&lt;resource&gt;/lvanlys.dll"/>
		</Item>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
