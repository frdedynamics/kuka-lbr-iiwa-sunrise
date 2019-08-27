package com.gates;


import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class UdpCartesianControl extends RoboticsAPIApplication {
	
	private DatagramSocket ds;
	private InetAddress ip;
	
	
	
	private String externalPcIp;
	private int udpPort;

	@Inject
	private LBR lbr;

	@Override
	public void initialize() {
		// initialize your application here
		getLogger().info("Initialize..");
		
		externalPcIp = "172.31.1.150";
		udpPort = 30001;
		
		getLogger().info("Create datagram socket");
		try {
			ds = new DatagramSocket();
			ds.setSoTimeout(30000);
		} catch (SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}  
		
		getLogger().info("Address..");
		try {
			ip = InetAddress.getByName(externalPcIp);
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}  
		getLogger().info(ip.toString());
		getLogger().info("Initialized.");
		
	}
	

	@Override
	public void run() {
		// your application execution starts here
		getLogger().info("Get and send current frame.");
	    Frame currentFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
	    getLogger().info(currentFrame.toString());
	    
	    sendFrame(currentFrame, ds, ip, udpPort);
	    
	    getLogger().info("Get new command frame.");
	    Frame commandFrame = getCommandFrame(currentFrame, ds);
	    getLogger().info(commandFrame.toString());
	    
	    getLogger().info("Close connection to client");
	    ds.close();  
	    
	}
	
	private void sendFrame(Frame _currentFrame, DatagramSocket _ds, InetAddress _ip, int _port){
		String str = _currentFrame.toString();
	    DatagramPacket dp = new DatagramPacket(str.getBytes(), str.length(), _ip, _port);  
	    try {
			_ds.send(dp);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}  
	}
	
	private Frame getCommandFrame(Frame _currentFrame, DatagramSocket _ds){
		Frame commandFrame = _currentFrame;
		byte[] buf = new byte[256];
		
		DatagramPacket dp = new DatagramPacket(buf, buf.length);
        try {
			_ds.receive(dp);
			String received = new String(dp.getData(), 0, dp.getLength());
			if (received != null && !received.isEmpty()){
				commandFrame = parseFrameCommand(received, _currentFrame);
			}
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        
        return commandFrame;
	}
	
	private Frame parseFrameCommand(String str, Frame _currentFrame){
		boolean isSane = true;
		Frame commandFrame = _currentFrame.copyWithRedundancy();
		
		String[] strArray = str.split("\\s+");
        int strIndex = 0;
        int strIndexInner = 0;
        for (String a : strArray){
            if (strIndex == 0){
                Double X = Double.parseDouble(a.substring(3));
                commandFrame.setX(X);
            }else if (strIndex == 5){
                Double C = Double.parseDouble(a.substring(2, a.length()-1));
                commandFrame.setGammaRad(C);
            }else {
                if (strIndexInner == 0){
                    Double Y = Double.parseDouble(a.substring(2));
                    commandFrame.setY(Y);
                }
                if (strIndexInner == 1){
                    Double Z = Double.parseDouble(a.substring(2));
                    commandFrame.setZ(Z);
                }
                if (strIndexInner == 2){
                    Double A = Double.parseDouble(a.substring(2));
                    commandFrame.setAlphaRad(A);
                }
                if (strIndexInner == 3){
                    Double B = Double.parseDouble(a.substring(2));
                    commandFrame.setBetaRad(B);
                }

                strIndexInner++;
            }
            strIndex++;
        }
        
        if (isSane){
        	return commandFrame;
        }else {
        	return _currentFrame;
        }
		
		
	}
}