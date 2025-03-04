/*
* MATLAB Compiler: 24.2 (R2024b)
* Date: Wed Oct 23 17:19:13 2024
* Arguments:
* "-B""macro_default""-W""dotnet:MatlabControlLib,MatlabControlLib,4.0,private,version=1.5
* ""-T""link:lib""-d""D:\alber\Documents\Cosas de la
* Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoPropio\C
* ontrol\MatlabControlLib\for_testing""-v""class{MatlabControlLib:D:\alber\Documents\Cosas
* de la
* Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoPropio\C
* ontrol\SingleTransportationControl2D.m,D:\alber\Documents\Cosas de la
* Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoPropio\C
* ontrol\TransportationControl2D.m,D:\alber\Documents\Cosas de la
* Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoPropio\C
* ontrol\TransportationControl2D_Debug.m,D:\alber\Documents\Cosas de la
* Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoPropio\C
* ontrol\TransportationControl3D.m,D:\alber\Documents\Cosas de la
* Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoPropio\C
* ontrol\TransportationControl3D_Debug.m}"
*/
using System;
using System.Reflection;
using System.IO;
using MathWorks.MATLAB.NET.Arrays;
using MathWorks.MATLAB.NET.Utility;

#if SHARED
[assembly: System.Reflection.AssemblyKeyFile(@"")]
#endif

namespace MatlabControlLib
{

  /// <summary>
  /// The MatlabControlLib class provides a CLS compliant, MWArray interface to the
  /// MATLAB functions contained in the files:
  /// <newpara></newpara>
  /// D:\alber\Documents\Cosas de la
  /// Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoProp
  /// io\Control\SingleTransportationControl2D.m
  /// <newpara></newpara>
  /// D:\alber\Documents\Cosas de la
  /// Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoProp
  /// io\Control\TransportationControl2D.m
  /// <newpara></newpara>
  /// D:\alber\Documents\Cosas de la
  /// Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoProp
  /// io\Control\TransportationControl2D_Debug.m
  /// <newpara></newpara>
  /// D:\alber\Documents\Cosas de la
  /// Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoProp
  /// io\Control\TransportationControl3D.m
  /// <newpara></newpara>
  /// D:\alber\Documents\Cosas de la
  /// Universidad\Apuntes\Master\Internships\DeRobSim\ThirdParty\TFG_Robotarium\CodigoProp
  /// io\Control\TransportationControl3D_Debug.m
  /// </summary>
  /// <remarks>
  /// @Version 1.5
  /// </remarks>
  public class MatlabControlLib : IDisposable
  {
    #region Constructors

    /// <summary internal= "true">
    /// The static constructor instantiates and initializes the MATLAB Runtime instance.
    /// </summary>
    static MatlabControlLib()
    {
      if (MWMCR.MCRAppInitialized)
      {
        try
        {
          System.Reflection.Assembly assembly = System.Reflection.Assembly.GetExecutingAssembly();

          string ctfFilePath= assembly.Location;

		  int lastDelimiter = ctfFilePath.LastIndexOf(@"/");

	      if (lastDelimiter == -1)
		  {
		    lastDelimiter = ctfFilePath.LastIndexOf(@"\");
		  }

          ctfFilePath= ctfFilePath.Remove(lastDelimiter, (ctfFilePath.Length - lastDelimiter));

          string ctfFileName = "MatlabControlLib.ctf";

          Stream embeddedCtfStream = null;

          String[] resourceStrings = assembly.GetManifestResourceNames();

          foreach (String name in resourceStrings)
          {
            if (name.Contains(ctfFileName))
            {
              embeddedCtfStream = assembly.GetManifestResourceStream(name);
              break;
            }
          }
          mcr= new MWMCR("",
                         ctfFilePath, embeddedCtfStream, true);
        }
        catch(Exception ex)
        {
          ex_ = new Exception("MWArray assembly failed to be initialized", ex);
        }
      }
      else
      {
        ex_ = new ApplicationException("MWArray assembly could not be initialized");
      }
    }


    /// <summary>
    /// Constructs a new instance of the MatlabControlLib class.
    /// </summary>
    public MatlabControlLib()
    {
      if(ex_ != null)
      {
        throw ex_;
      }
    }


    #endregion Constructors

    #region Finalize

    /// <summary internal= "true">
    /// Class destructor called by the CLR garbage collector.
    /// </summary>
    ~MatlabControlLib()
    {
      Dispose(false);
    }


    /// <summary>
    /// Frees the native resources associated with this object
    /// </summary>
    public void Dispose()
    {
      Dispose(true);

      GC.SuppressFinalize(this);
    }


    /// <summary internal= "true">
    /// Internal dispose function
    /// </summary>
    protected virtual void Dispose(bool disposing)
    {
      if (!disposed)
      {
        disposed= true;

        if (disposing)
        {
          // Free managed resources;
        }

        // Free native resources
      }
    }


    #endregion Finalize

    #region Methods

    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D()
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos);
    }


    /// <summary>
    /// Provides a single output, 5-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H);
    }


    /// <summary>
    /// Provides a single output, 6-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H);
    }


    /// <summary>
    /// Provides a single output, 7-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides a single output, 8-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides a single output, 9-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides a single output, 10-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides a single output, 11-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides a single output, 12-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides a single output, 13-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides a single output, 14-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides a single output, 15-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides a single output, 16-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <param name="alpha_G">Input argument #16</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H, MWArray alpha_G)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides a single output, 17-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <param name="alpha_G">Input argument #16</param>
    /// <param name="dt">Input argument #17</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H, MWArray alpha_G, MWArray dt)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides a single output, 18-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <param name="alpha_G">Input argument #16</param>
    /// <param name="dt">Input argument #17</param>
    /// <param name="sd">Input argument #18</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H, MWArray alpha_G, MWArray dt, 
                                           MWArray sd)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides a single output, 19-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <param name="alpha_G">Input argument #16</param>
    /// <param name="dt">Input argument #17</param>
    /// <param name="sd">Input argument #18</param>
    /// <param name="thd">Input argument #19</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H, MWArray alpha_G, MWArray dt, 
                                           MWArray sd, MWArray thd)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides a single output, 20-input MWArrayinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <param name="alpha_G">Input argument #16</param>
    /// <param name="dt">Input argument #17</param>
    /// <param name="sd">Input argument #18</param>
    /// <param name="thd">Input argument #19</param>
    /// <param name="asat">Input argument #20</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray SingleTransportationControl2D(MWArray robot_id, MWArray positions, 
                                           MWArray destinations, MWArray prev_pos, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H, MWArray alpha_G, MWArray dt, 
                                           MWArray sd, MWArray thd, MWArray asat)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H);
    }


    /// <summary>
    /// Provides the standard 7-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides the standard 8-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides the standard 9-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides the standard 10-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides the standard 11-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides the standard 12-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides the standard 13-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides the standard 14-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides the standard 15-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides the standard 16-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <param name="alpha_G">Input argument #16</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                             MWArray alpha_G)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides the standard 17-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <param name="alpha_G">Input argument #16</param>
    /// <param name="dt">Input argument #17</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                             MWArray alpha_G, MWArray dt)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides the standard 18-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <param name="alpha_G">Input argument #16</param>
    /// <param name="dt">Input argument #17</param>
    /// <param name="sd">Input argument #18</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                             MWArray alpha_G, MWArray dt, MWArray sd)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides the standard 19-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <param name="alpha_G">Input argument #16</param>
    /// <param name="dt">Input argument #17</param>
    /// <param name="sd">Input argument #18</param>
    /// <param name="thd">Input argument #19</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                             MWArray alpha_G, MWArray dt, MWArray sd, 
                                             MWArray thd)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides the standard 20-input MWArray interface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <param name="prev_pos">Input argument #4</param>
    /// <param name="k1H">Input argument #5</param>
    /// <param name="k2H">Input argument #6</param>
    /// <param name="k1G">Input argument #7</param>
    /// <param name="k2G">Input argument #8</param>
    /// <param name="k1s">Input argument #9</param>
    /// <param name="k2s">Input argument #10</param>
    /// <param name="k1g">Input argument #11</param>
    /// <param name="k2g">Input argument #12</param>
    /// <param name="k1th">Input argument #13</param>
    /// <param name="k2th">Input argument #14</param>
    /// <param name="alpha_H">Input argument #15</param>
    /// <param name="alpha_G">Input argument #16</param>
    /// <param name="dt">Input argument #17</param>
    /// <param name="sd">Input argument #18</param>
    /// <param name="thd">Input argument #19</param>
    /// <param name="asat">Input argument #20</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] SingleTransportationControl2D(int numArgsOut, MWArray robot_id, 
                                             MWArray positions, MWArray destinations, 
                                             MWArray prev_pos, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                             MWArray alpha_G, MWArray dt, MWArray sd, 
                                             MWArray thd, MWArray asat)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides an interface for the SingleTransportationControl2D function in which the
    /// input and output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void SingleTransportationControl2D(int numArgsOut, ref MWArray[] argsOut, 
                                    MWArray[] argsIn)
    {
      mcr.EvaluateFunction("SingleTransportationControl2D", numArgsOut, ref argsOut, 
                                    argsIn);
    }


    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D()
    {
      return mcr.EvaluateFunction("TransportationControl2D", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H);
    }


    /// <summary>
    /// Provides a single output, 5-input MWArrayinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H);
    }


    /// <summary>
    /// Provides a single output, 6-input MWArrayinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides a single output, 7-input MWArrayinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides a single output, 8-input MWArrayinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides a single output, 9-input MWArrayinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides a single output, 10-input MWArrayinterface to the
    /// TransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s, MWArray 
                                     k1g)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides a single output, 11-input MWArrayinterface to the
    /// TransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s, MWArray 
                                     k1g, MWArray k2g)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides a single output, 12-input MWArrayinterface to the
    /// TransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s, MWArray 
                                     k1g, MWArray k2g, MWArray k1th)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides a single output, 13-input MWArrayinterface to the
    /// TransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s, MWArray 
                                     k1g, MWArray k2g, MWArray k1th, MWArray k2th)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides a single output, 14-input MWArrayinterface to the
    /// TransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s, MWArray 
                                     k1g, MWArray k2g, MWArray k1th, MWArray k2th, 
                                     MWArray alpha_H)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides a single output, 15-input MWArrayinterface to the
    /// TransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s, MWArray 
                                     k1g, MWArray k2g, MWArray k1th, MWArray k2th, 
                                     MWArray alpha_H, MWArray alpha_G)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides a single output, 16-input MWArrayinterface to the
    /// TransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s, MWArray 
                                     k1g, MWArray k2g, MWArray k1th, MWArray k2th, 
                                     MWArray alpha_H, MWArray alpha_G, MWArray dt)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides a single output, 17-input MWArrayinterface to the
    /// TransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s, MWArray 
                                     k1g, MWArray k2g, MWArray k1th, MWArray k2th, 
                                     MWArray alpha_H, MWArray alpha_G, MWArray dt, 
                                     MWArray sd)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides a single output, 18-input MWArrayinterface to the
    /// TransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <param name="thd">Input argument #18</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s, MWArray 
                                     k1g, MWArray k2g, MWArray k1th, MWArray k2th, 
                                     MWArray alpha_H, MWArray alpha_G, MWArray dt, 
                                     MWArray sd, MWArray thd)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides a single output, 19-input MWArrayinterface to the
    /// TransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <param name="thd">Input argument #18</param>
    /// <param name="asat">Input argument #19</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D(MWArray positions, MWArray destinations, 
                                     MWArray prev_pos, MWArray k1H, MWArray k2H, MWArray 
                                     k1G, MWArray k2G, MWArray k1s, MWArray k2s, MWArray 
                                     k1g, MWArray k2g, MWArray k1th, MWArray k2th, 
                                     MWArray alpha_H, MWArray alpha_G, MWArray dt, 
                                     MWArray sd, MWArray thd, MWArray asat)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides the standard 7-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides the standard 8-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray k1s)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides the standard 9-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides the standard 10-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s, MWArray k1g)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides the standard 11-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s, MWArray k1g, MWArray k2g)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides the standard 12-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s, MWArray k1g, MWArray k2g, 
                                       MWArray k1th)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides the standard 13-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s, MWArray k1g, MWArray k2g, 
                                       MWArray k1th, MWArray k2th)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides the standard 14-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s, MWArray k1g, MWArray k2g, 
                                       MWArray k1th, MWArray k2th, MWArray alpha_H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides the standard 15-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s, MWArray k1g, MWArray k2g, 
                                       MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                       MWArray alpha_G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides the standard 16-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s, MWArray k1g, MWArray k2g, 
                                       MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                       MWArray alpha_G, MWArray dt)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides the standard 17-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s, MWArray k1g, MWArray k2g, 
                                       MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                       MWArray alpha_G, MWArray dt, MWArray sd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides the standard 18-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <param name="thd">Input argument #18</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s, MWArray k1g, MWArray k2g, 
                                       MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                       MWArray alpha_G, MWArray dt, MWArray sd, MWArray 
                                       thd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides the standard 19-input MWArray interface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <param name="thd">Input argument #18</param>
    /// <param name="asat">Input argument #19</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray prev_pos, MWArray k1H, 
                                       MWArray k2H, MWArray k1G, MWArray k2G, MWArray 
                                       k1s, MWArray k2s, MWArray k1g, MWArray k2g, 
                                       MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                       MWArray alpha_G, MWArray dt, MWArray sd, MWArray 
                                       thd, MWArray asat)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides an interface for the TransportationControl2D function in which the input
    /// and output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void TransportationControl2D(int numArgsOut, ref MWArray[] argsOut, MWArray[] 
                              argsIn)
    {
      mcr.EvaluateFunction("TransportationControl2D", numArgsOut, ref argsOut, argsIn);
    }


    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug()
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H);
    }


    /// <summary>
    /// Provides a single output, 5-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H);
    }


    /// <summary>
    /// Provides a single output, 6-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides a single output, 7-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides a single output, 8-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides a single output, 9-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides a single output, 10-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides a single output, 11-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides a single output, 12-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides a single output, 13-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides a single output, 14-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides a single output, 15-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H, MWArray alpha_G)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides a single output, 16-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H, MWArray alpha_G, MWArray dt)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides a single output, 17-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H, MWArray alpha_G, MWArray dt, 
                                           MWArray sd)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides a single output, 18-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <param name="thd">Input argument #18</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H, MWArray alpha_G, MWArray dt, 
                                           MWArray sd, MWArray thd)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides a single output, 19-input MWArrayinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <param name="thd">Input argument #18</param>
    /// <param name="asat">Input argument #19</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl2D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray prev_pos_in1, 
                                           MWArray k1H, MWArray k2H, MWArray k1G, MWArray 
                                           k2G, MWArray k1s, MWArray k2s, MWArray k1g, 
                                           MWArray k2g, MWArray k1th, MWArray k2th, 
                                           MWArray alpha_H, MWArray alpha_G, MWArray dt, 
                                           MWArray sd, MWArray thd, MWArray asat)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides the standard 7-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides the standard 8-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides the standard 9-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides the standard 10-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides the standard 11-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides the standard 12-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides the standard 13-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides the standard 14-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides the standard 15-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                             MWArray alpha_G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides the standard 16-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                             MWArray alpha_G, MWArray dt)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides the standard 17-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                             MWArray alpha_G, MWArray dt, MWArray sd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides the standard 18-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <param name="thd">Input argument #18</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                             MWArray alpha_G, MWArray dt, MWArray sd, 
                                             MWArray thd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides the standard 19-input MWArray interface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <param name="k1H">Input argument #4</param>
    /// <param name="k2H">Input argument #5</param>
    /// <param name="k1G">Input argument #6</param>
    /// <param name="k2G">Input argument #7</param>
    /// <param name="k1s">Input argument #8</param>
    /// <param name="k2s">Input argument #9</param>
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <param name="thd">Input argument #18</param>
    /// <param name="asat">Input argument #19</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl2D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray 
                                             prev_pos_in1, MWArray k1H, MWArray k2H, 
                                             MWArray k1G, MWArray k2G, MWArray k1s, 
                                             MWArray k2s, MWArray k1g, MWArray k2g, 
                                             MWArray k1th, MWArray k2th, MWArray alpha_H, 
                                             MWArray alpha_G, MWArray dt, MWArray sd, 
                                             MWArray thd, MWArray asat)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides an interface for the TransportationControl2D_Debug function in which the
    /// input and output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void TransportationControl2D_Debug(int numArgsOut, ref MWArray[] argsOut, 
                                    MWArray[] argsIn)
    {
      mcr.EvaluateFunction("TransportationControl2D_Debug", numArgsOut, ref argsOut, 
                                    argsIn);
    }


    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D()
    {
      return mcr.EvaluateFunction("TransportationControl3D", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D(MWArray positions)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D(MWArray positions, MWArray destinations)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D(MWArray positions, MWArray destinations, 
                                     MWArray kH)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D(MWArray positions, MWArray destinations, 
                                     MWArray kH, MWArray kG)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG);
    }


    /// <summary>
    /// Provides a single output, 5-input MWArrayinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D(MWArray positions, MWArray destinations, 
                                     MWArray kH, MWArray kG, MWArray ks)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks);
    }


    /// <summary>
    /// Provides a single output, 6-input MWArrayinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D(MWArray positions, MWArray destinations, 
                                     MWArray kH, MWArray kG, MWArray ks, MWArray kg)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks, kg);
    }


    /// <summary>
    /// Provides a single output, 7-input MWArrayinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D(MWArray positions, MWArray destinations, 
                                     MWArray kH, MWArray kG, MWArray ks, MWArray kg, 
                                     MWArray kth)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth);
    }


    /// <summary>
    /// Provides a single output, 8-input MWArrayinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D(MWArray positions, MWArray destinations, 
                                     MWArray kH, MWArray kG, MWArray ks, MWArray kg, 
                                     MWArray kth, MWArray sd)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd);
    }


    /// <summary>
    /// Provides a single output, 9-input MWArrayinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <param name="thd">Input argument #9</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D(MWArray positions, MWArray destinations, 
                                     MWArray kH, MWArray kG, MWArray ks, MWArray kg, 
                                     MWArray kth, MWArray sd, MWArray thd)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd, thd);
    }


    /// <summary>
    /// Provides a single output, 10-input MWArrayinterface to the
    /// TransportationControl3D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <param name="thd">Input argument #9</param>
    /// <param name="vsat">Input argument #10</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D(MWArray positions, MWArray destinations, 
                                     MWArray kH, MWArray kG, MWArray ks, MWArray kg, 
                                     MWArray kth, MWArray sd, MWArray thd, MWArray vsat)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd, thd, vsat);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut, MWArray positions)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut, MWArray positions, MWArray 
                                       destinations)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray kH)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray kH, MWArray kG)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray kH, MWArray kG, MWArray ks)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray kH, MWArray kG, MWArray ks, 
                                       MWArray kg)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks, kg);
    }


    /// <summary>
    /// Provides the standard 7-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray kH, MWArray kG, MWArray ks, 
                                       MWArray kg, MWArray kth)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth);
    }


    /// <summary>
    /// Provides the standard 8-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray kH, MWArray kG, MWArray ks, 
                                       MWArray kg, MWArray kth, MWArray sd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd);
    }


    /// <summary>
    /// Provides the standard 9-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <param name="thd">Input argument #9</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray kH, MWArray kG, MWArray ks, 
                                       MWArray kg, MWArray kth, MWArray sd, MWArray thd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd, thd);
    }


    /// <summary>
    /// Provides the standard 10-input MWArray interface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <param name="thd">Input argument #9</param>
    /// <param name="vsat">Input argument #10</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D(int numArgsOut, MWArray positions, MWArray 
                                       destinations, MWArray kH, MWArray kG, MWArray ks, 
                                       MWArray kg, MWArray kth, MWArray sd, MWArray thd, 
                                       MWArray vsat)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd, thd, vsat);
    }


    /// <summary>
    /// Provides an interface for the TransportationControl3D function in which the input
    /// and output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void TransportationControl3D(int numArgsOut, ref MWArray[] argsOut, MWArray[] 
                              argsIn)
    {
      mcr.EvaluateFunction("TransportationControl3D", numArgsOut, ref argsOut, argsIn);
    }


    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug()
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug(MWArray positions_in1)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray kH)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray kH, MWArray kG)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG);
    }


    /// <summary>
    /// Provides a single output, 5-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray kH, MWArray kG, 
                                           MWArray ks)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks);
    }


    /// <summary>
    /// Provides a single output, 6-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray kH, MWArray kG, 
                                           MWArray ks, MWArray kg)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg);
    }


    /// <summary>
    /// Provides a single output, 7-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray kH, MWArray kG, 
                                           MWArray ks, MWArray kg, MWArray kth)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth);
    }


    /// <summary>
    /// Provides a single output, 8-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray kH, MWArray kG, 
                                           MWArray ks, MWArray kg, MWArray kth, MWArray 
                                           sd)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd);
    }


    /// <summary>
    /// Provides a single output, 9-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <param name="thd">Input argument #9</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray kH, MWArray kG, 
                                           MWArray ks, MWArray kg, MWArray kth, MWArray 
                                           sd, MWArray thd)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd, thd);
    }


    /// <summary>
    /// Provides a single output, 10-input MWArrayinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <param name="thd">Input argument #9</param>
    /// <param name="vsat">Input argument #10</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray TransportationControl3D_Debug(MWArray positions_in1, MWArray 
                                           destinations_in1, MWArray kH, MWArray kG, 
                                           MWArray ks, MWArray kg, MWArray kth, MWArray 
                                           sd, MWArray thd, MWArray vsat)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd, thd, vsat);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut, MWArray positions_in1)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray kH)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray kH, 
                                             MWArray kG)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray kH, 
                                             MWArray kG, MWArray ks)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray kH, 
                                             MWArray kG, MWArray ks, MWArray kg)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg);
    }


    /// <summary>
    /// Provides the standard 7-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray kH, 
                                             MWArray kG, MWArray ks, MWArray kg, MWArray 
                                             kth)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth);
    }


    /// <summary>
    /// Provides the standard 8-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray kH, 
                                             MWArray kG, MWArray ks, MWArray kg, MWArray 
                                             kth, MWArray sd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd);
    }


    /// <summary>
    /// Provides the standard 9-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <param name="thd">Input argument #9</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray kH, 
                                             MWArray kG, MWArray ks, MWArray kg, MWArray 
                                             kth, MWArray sd, MWArray thd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd, thd);
    }


    /// <summary>
    /// Provides the standard 10-input MWArray interface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <param name="kG">Input argument #4</param>
    /// <param name="ks">Input argument #5</param>
    /// <param name="kg">Input argument #6</param>
    /// <param name="kth">Input argument #7</param>
    /// <param name="sd">Input argument #8</param>
    /// <param name="thd">Input argument #9</param>
    /// <param name="vsat">Input argument #10</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] TransportationControl3D_Debug(int numArgsOut, MWArray positions_in1, 
                                             MWArray destinations_in1, MWArray kH, 
                                             MWArray kG, MWArray ks, MWArray kg, MWArray 
                                             kth, MWArray sd, MWArray thd, MWArray vsat)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd, thd, vsat);
    }


    /// <summary>
    /// Provides an interface for the TransportationControl3D_Debug function in which the
    /// input and output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void TransportationControl3D_Debug(int numArgsOut, ref MWArray[] argsOut, 
                                    MWArray[] argsIn)
    {
      mcr.EvaluateFunction("TransportationControl3D_Debug", numArgsOut, ref argsOut, 
                                    argsIn);
    }



    /// <summary>
    /// This method will cause a MATLAB figure window to behave as a modal dialog box.
    /// The method will not return until all the figure windows associated with this
    /// component have been closed.
    /// </summary>
    /// <remarks>
    /// An application should only call this method when required to keep the
    /// MATLAB figure window from disappearing.  Other techniques, such as calling
    /// Console.ReadLine() from the application should be considered where
    /// possible.</remarks>
    ///
    public void WaitForFiguresToDie()
    {
      mcr.WaitForFiguresToDie();
    }



    #endregion Methods

    #region Class Members

    private static MWMCR mcr= null;

    private static Exception ex_= null;

    private bool disposed= false;

    #endregion Class Members
  }
}
