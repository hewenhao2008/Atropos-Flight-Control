		function IMU(ala,cab,gin,alt,suelo0, suelo1){
			this.ala=ala;
			this.cab=cab;
			this.gin=gin;
			this.alt=alt;
			this.suelo0=suelo0;
			this.suelo1=suelo1;
		}
		function PID(Xcpos, Ycpos, Xtpos, Ytpos, Xerr, Yerr){
			this.Xcurr=Xcpos;
			this.Ycurr=Ycpos;
			this.Xtarget=Xtpos;
			this.Ytarget=Ytpos;
            this.Xerr=Xerr;
            this.Yerr=Yerr;
		}
		
		function GPS(latitud, longitud, altitud, enlace){
			this.longitud=longitud;
			this.latitud=latitud;
			this.altitud=altitud;
			this.enlace=enlace;
		}
