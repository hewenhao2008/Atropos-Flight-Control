		function getCanvas(canvasElement){
				var canvas = $(canvasElement);
				var ctx = canvas.getContext("2d");
				return ctx;
		}
		function rectangulo(canvasContext,lX,lY,sizeX,sizeY,rgbstring){
				var ctx=canvasContext;
				if (ctx){
						ctx.fillStyle = rgbstring;
						ctx.fillRect(lX, lY, sizeX, sizeY);
				}
		}
		function triangulo(canvasContext,lX,lY,base,altura,rgbstring, rgbstroke){
				var ctx=canvasContext;
				if (ctx){
                        if (rgbstroke==''){rgbstroke=rgbstring;}
						var bd2=base/2;
						var ad2=altura/2;						
						ctx.beginPath();
						ctx.moveTo(lX, lY-ad2);
						ctx.lineTo(lX+bd2, lY+ad2);
						ctx.lineTo(lX-(bd2), lY+(ad2));						
						ctx.lineTo(lX, lY-ad2);
						ctx.closePath();
						ctx.fillStyle = rgbstring;
						ctx.fill();
						ctx.strokeStyle =rgbstroke;
						ctx.stroke();
				}
		}		
		function linea(ctx, fromX, fromY, toX, toY, rgbstring){
				if (ctx){
					ctx.strokeStyle=rgbstring;
					ctx.beginPath();
					ctx.moveTo(fromX, fromY);
					ctx.lineTo(toX,toY);
					ctx.closePath();
					ctx.stroke();
				}
		}
		function cruz(ctx, centerX, centerY, size, rgbstring){
				linea(ctx, centerX-(size/2),centerY,centerX+(size/2),centerY,rgbstring );
				linea(ctx, centerX,centerY-(size/2),centerX,centerY+(size/2),rgbstring );
		}		
		function arco(ctx,x,y,radius,startAngle,endAngle, clockwise, rgbstring){
			if (ctx){
				ctx.strokeStyle=rgbstring;
				ctx.beginPath();

				ctx.arc(x,y,radius,startAngle,endAngle, clockwise);
			
				ctx.stroke();
				ctx.closePath();
			}
		}
		function setCanvasProperties(viewContext){
			viewContext.shadowOffsetX = 2;
			viewContext.shadowOffsetY = 2;
			viewContext.shadowBlur = 4;
			viewContext.shadowColor = "rgba(0, 0, 0, 3)";	
			viewContext.lineWidth=2;			
		}
		function setFlatCanvasProperties(viewContext){
			viewContext.shadowOffsetX = 0;
			viewContext.shadowOffsetY = 0;
			viewContext.shadowBlur = 0;
			viewContext.shadowColor = "rgba(0, 0, 0, 100)";	
			viewContext.lineWidth=2;			
		}			
