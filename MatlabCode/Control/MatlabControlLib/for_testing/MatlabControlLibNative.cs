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

namespace MatlabControlLibNative
{

  /// <summary>
  /// The MatlabControlLib class provides a CLS compliant, Object (native) interface to
  /// the MATLAB functions contained in the files:
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
    /// Provides a single output, 0-input Objectinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D()
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the
    /// SingleTransportationControl2D MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="robot_id">Input argument #1</param>
    /// <param name="positions">Input argument #2</param>
    /// <param name="destinations">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos);
    }


    /// <summary>
    /// Provides a single output, 5-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H);
    }


    /// <summary>
    /// Provides a single output, 6-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H);
    }


    /// <summary>
    /// Provides a single output, 7-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides a single output, 8-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides a single output, 9-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides a single output, 10-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides a single output, 11-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s, Object k1g)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides a single output, 12-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s, Object k1g, Object k2g)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides a single output, 13-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s, Object k1g, Object k2g, Object k1th)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides a single output, 14-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides a single output, 15-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides a single output, 16-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H, Object 
                                          alpha_G)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides a single output, 17-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H, Object 
                                          alpha_G, Object dt)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides a single output, 18-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H, Object 
                                          alpha_G, Object dt, Object sd)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides a single output, 19-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H, Object 
                                          alpha_G, Object dt, Object sd, Object thd)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides a single output, 20-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object SingleTransportationControl2D(Object robot_id, Object positions, Object 
                                          destinations, Object prev_pos, Object k1H, 
                                          Object k2H, Object k1G, Object k2G, Object k1s, 
                                          Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H, Object 
                                          alpha_G, Object dt, Object sd, Object thd, 
                                          Object asat)
    {
      return mcr.EvaluateFunction("SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H);
    }


    /// <summary>
    /// Provides the standard 7-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides the standard 8-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides the standard 9-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides the standard 10-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides the standard 11-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s, Object 
                                            k1g)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides the standard 12-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s, Object 
                                            k1g, Object k2g)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides the standard 13-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s, Object 
                                            k1g, Object k2g, Object k1th)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides the standard 14-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s, Object 
                                            k1g, Object k2g, Object k1th, Object k2th)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides the standard 15-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s, Object 
                                            k1g, Object k2g, Object k1th, Object k2th, 
                                            Object alpha_H)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides the standard 16-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s, Object 
                                            k1g, Object k2g, Object k1th, Object k2th, 
                                            Object alpha_H, Object alpha_G)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides the standard 17-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s, Object 
                                            k1g, Object k2g, Object k1th, Object k2th, 
                                            Object alpha_H, Object alpha_G, Object dt)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides the standard 18-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s, Object 
                                            k1g, Object k2g, Object k1th, Object k2th, 
                                            Object alpha_H, Object alpha_G, Object dt, 
                                            Object sd)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides the standard 19-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s, Object 
                                            k1g, Object k2g, Object k1th, Object k2th, 
                                            Object alpha_H, Object alpha_G, Object dt, 
                                            Object sd, Object thd)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides the standard 20-input Object interface to the
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
    public Object[] SingleTransportationControl2D(int numArgsOut, Object robot_id, Object 
                                            positions, Object destinations, Object 
                                            prev_pos, Object k1H, Object k2H, Object k1G, 
                                            Object k2G, Object k1s, Object k2s, Object 
                                            k1g, Object k2g, Object k1th, Object k2th, 
                                            Object alpha_H, Object alpha_G, Object dt, 
                                            Object sd, Object thd, Object asat)
    {
      return mcr.EvaluateFunction(numArgsOut, "SingleTransportationControl2D", robot_id, positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides an interface for the SingleTransportationControl2D function in which the
    /// input and output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("SingleTransportationControl2D", 20, 1, 0)]
    protected void SingleTransportationControl2D(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("SingleTransportationControl2D", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }
    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D()
    {
      return mcr.EvaluateFunction("TransportationControl2D", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the TransportationControl2D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="prev_pos">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the TransportationControl2D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H);
    }


    /// <summary>
    /// Provides a single output, 5-input Objectinterface to the TransportationControl2D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H);
    }


    /// <summary>
    /// Provides a single output, 6-input Objectinterface to the TransportationControl2D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides a single output, 7-input Objectinterface to the TransportationControl2D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides a single output, 8-input Objectinterface to the TransportationControl2D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides a single output, 9-input Objectinterface to the TransportationControl2D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides a single output, 10-input Objectinterface to the TransportationControl2D
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
    /// <param name="k1g">Input argument #10</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s, Object k1g)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides a single output, 11-input Objectinterface to the TransportationControl2D
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
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s, Object k1g, Object k2g)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides a single output, 12-input Objectinterface to the TransportationControl2D
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
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s, Object k1g, Object k2g, 
                                    Object k1th)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides a single output, 13-input Objectinterface to the TransportationControl2D
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
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s, Object k1g, Object k2g, 
                                    Object k1th, Object k2th)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides a single output, 14-input Objectinterface to the TransportationControl2D
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
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s, Object k1g, Object k2g, 
                                    Object k1th, Object k2th, Object alpha_H)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides a single output, 15-input Objectinterface to the TransportationControl2D
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
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s, Object k1g, Object k2g, 
                                    Object k1th, Object k2th, Object alpha_H, Object 
                                    alpha_G)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides a single output, 16-input Objectinterface to the TransportationControl2D
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
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s, Object k1g, Object k2g, 
                                    Object k1th, Object k2th, Object alpha_H, Object 
                                    alpha_G, Object dt)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides a single output, 17-input Objectinterface to the TransportationControl2D
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
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s, Object k1g, Object k2g, 
                                    Object k1th, Object k2th, Object alpha_H, Object 
                                    alpha_G, Object dt, Object sd)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides a single output, 18-input Objectinterface to the TransportationControl2D
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
    /// <param name="k1g">Input argument #10</param>
    /// <param name="k2g">Input argument #11</param>
    /// <param name="k1th">Input argument #12</param>
    /// <param name="k2th">Input argument #13</param>
    /// <param name="alpha_H">Input argument #14</param>
    /// <param name="alpha_G">Input argument #15</param>
    /// <param name="dt">Input argument #16</param>
    /// <param name="sd">Input argument #17</param>
    /// <param name="thd">Input argument #18</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s, Object k1g, Object k2g, 
                                    Object k1th, Object k2th, Object alpha_H, Object 
                                    alpha_G, Object dt, Object sd, Object thd)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides a single output, 19-input Objectinterface to the TransportationControl2D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D(Object positions, Object destinations, Object 
                                    prev_pos, Object k1H, Object k2H, Object k1G, Object 
                                    k2G, Object k1s, Object k2s, Object k1g, Object k2g, 
                                    Object k1th, Object k2th, Object alpha_H, Object 
                                    alpha_G, Object dt, Object sd, Object thd, Object 
                                    asat)
    {
      return mcr.EvaluateFunction("TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides the standard 7-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides the standard 8-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides the standard 9-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object k2s)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides the standard 10-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object 
                                      k2s, Object k1g)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides the standard 11-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object 
                                      k2s, Object k1g, Object k2g)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides the standard 12-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object 
                                      k2s, Object k1g, Object k2g, Object k1th)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides the standard 13-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object 
                                      k2s, Object k1g, Object k2g, Object k1th, Object 
                                      k2th)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides the standard 14-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object 
                                      k2s, Object k1g, Object k2g, Object k1th, Object 
                                      k2th, Object alpha_H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides the standard 15-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object 
                                      k2s, Object k1g, Object k2g, Object k1th, Object 
                                      k2th, Object alpha_H, Object alpha_G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides the standard 16-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object 
                                      k2s, Object k1g, Object k2g, Object k1th, Object 
                                      k2th, Object alpha_H, Object alpha_G, Object dt)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides the standard 17-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object 
                                      k2s, Object k1g, Object k2g, Object k1th, Object 
                                      k2th, Object alpha_H, Object alpha_G, Object dt, 
                                      Object sd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides the standard 18-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object 
                                      k2s, Object k1g, Object k2g, Object k1th, Object 
                                      k2th, Object alpha_H, Object alpha_G, Object dt, 
                                      Object sd, Object thd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides the standard 19-input Object interface to the TransportationControl2D
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
    public Object[] TransportationControl2D(int numArgsOut, Object positions, Object 
                                      destinations, Object prev_pos, Object k1H, Object 
                                      k2H, Object k1G, Object k2G, Object k1s, Object 
                                      k2s, Object k1g, Object k2g, Object k1th, Object 
                                      k2th, Object alpha_H, Object alpha_G, Object dt, 
                                      Object sd, Object thd, Object asat)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D", positions, destinations, prev_pos, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides an interface for the TransportationControl2D function in which the input
    /// and output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("TransportationControl2D", 19, 1, 0)]
    protected void TransportationControl2D(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("TransportationControl2D", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }
    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug()
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the
    /// TransportationControl2D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="prev_pos_in1">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H);
    }


    /// <summary>
    /// Provides a single output, 5-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H);
    }


    /// <summary>
    /// Provides a single output, 6-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides a single output, 7-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides a single output, 8-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides a single output, 9-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides a single output, 10-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s, Object k1g)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides a single output, 11-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s, Object k1g, Object k2g)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides a single output, 12-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s, Object k1g, Object k2g, Object 
                                          k1th)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides a single output, 13-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides a single output, 14-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides a single output, 15-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H, Object 
                                          alpha_G)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides a single output, 16-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H, Object 
                                          alpha_G, Object dt)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides a single output, 17-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H, Object 
                                          alpha_G, Object dt, Object sd)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides a single output, 18-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H, Object 
                                          alpha_G, Object dt, Object sd, Object thd)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides a single output, 19-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl2D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object prev_pos_in1, Object 
                                          k1H, Object k2H, Object k1G, Object k2G, Object 
                                          k1s, Object k2s, Object k1g, Object k2g, Object 
                                          k1th, Object k2th, Object alpha_H, Object 
                                          alpha_G, Object dt, Object sd, Object thd, 
                                          Object asat)
    {
      return mcr.EvaluateFunction("TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G);
    }


    /// <summary>
    /// Provides the standard 7-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G);
    }


    /// <summary>
    /// Provides the standard 8-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s);
    }


    /// <summary>
    /// Provides the standard 9-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s);
    }


    /// <summary>
    /// Provides the standard 10-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s, Object k1g)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g);
    }


    /// <summary>
    /// Provides the standard 11-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s, Object k1g, 
                                            Object k2g)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g);
    }


    /// <summary>
    /// Provides the standard 12-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s, Object k1g, 
                                            Object k2g, Object k1th)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th);
    }


    /// <summary>
    /// Provides the standard 13-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s, Object k1g, 
                                            Object k2g, Object k1th, Object k2th)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th);
    }


    /// <summary>
    /// Provides the standard 14-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s, Object k1g, 
                                            Object k2g, Object k1th, Object k2th, Object 
                                            alpha_H)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H);
    }


    /// <summary>
    /// Provides the standard 15-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s, Object k1g, 
                                            Object k2g, Object k1th, Object k2th, Object 
                                            alpha_H, Object alpha_G)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G);
    }


    /// <summary>
    /// Provides the standard 16-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s, Object k1g, 
                                            Object k2g, Object k1th, Object k2th, Object 
                                            alpha_H, Object alpha_G, Object dt)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt);
    }


    /// <summary>
    /// Provides the standard 17-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s, Object k1g, 
                                            Object k2g, Object k1th, Object k2th, Object 
                                            alpha_H, Object alpha_G, Object dt, Object sd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd);
    }


    /// <summary>
    /// Provides the standard 18-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s, Object k1g, 
                                            Object k2g, Object k1th, Object k2th, Object 
                                            alpha_H, Object alpha_G, Object dt, Object 
                                            sd, Object thd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd);
    }


    /// <summary>
    /// Provides the standard 19-input Object interface to the
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
    public Object[] TransportationControl2D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object prev_pos_in1, 
                                            Object k1H, Object k2H, Object k1G, Object 
                                            k2G, Object k1s, Object k2s, Object k1g, 
                                            Object k2g, Object k1th, Object k2th, Object 
                                            alpha_H, Object alpha_G, Object dt, Object 
                                            sd, Object thd, Object asat)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl2D_Debug", positions_in1, destinations_in1, prev_pos_in1, k1H, k2H, k1G, k2G, k1s, k2s, k1g, k2g, k1th, k2th, alpha_H, alpha_G, dt, sd, thd, asat);
    }


    /// <summary>
    /// Provides an interface for the TransportationControl2D_Debug function in which the
    /// input and output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("TransportationControl2D_Debug", 19, 16, 0)]
    protected void TransportationControl2D_Debug(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("TransportationControl2D_Debug", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }
    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D()
    {
      return mcr.EvaluateFunction("TransportationControl3D", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D(Object positions)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D(Object positions, Object destinations)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the TransportationControl3D
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions">Input argument #1</param>
    /// <param name="destinations">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D(Object positions, Object destinations, Object 
                                    kH)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the TransportationControl3D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D(Object positions, Object destinations, Object 
                                    kH, Object kG)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG);
    }


    /// <summary>
    /// Provides a single output, 5-input Objectinterface to the TransportationControl3D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D(Object positions, Object destinations, Object 
                                    kH, Object kG, Object ks)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks);
    }


    /// <summary>
    /// Provides a single output, 6-input Objectinterface to the TransportationControl3D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D(Object positions, Object destinations, Object 
                                    kH, Object kG, Object ks, Object kg)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks, kg);
    }


    /// <summary>
    /// Provides a single output, 7-input Objectinterface to the TransportationControl3D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D(Object positions, Object destinations, Object 
                                    kH, Object kG, Object ks, Object kg, Object kth)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth);
    }


    /// <summary>
    /// Provides a single output, 8-input Objectinterface to the TransportationControl3D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D(Object positions, Object destinations, Object 
                                    kH, Object kG, Object ks, Object kg, Object kth, 
                                    Object sd)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd);
    }


    /// <summary>
    /// Provides a single output, 9-input Objectinterface to the TransportationControl3D
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D(Object positions, Object destinations, Object 
                                    kH, Object kG, Object ks, Object kg, Object kth, 
                                    Object sd, Object thd)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd, thd);
    }


    /// <summary>
    /// Provides a single output, 10-input Objectinterface to the TransportationControl3D
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
    /// <param name="vsat">Input argument #10</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D(Object positions, Object destinations, Object 
                                    kH, Object kG, Object ks, Object kg, Object kth, 
                                    Object sd, Object thd, Object vsat)
    {
      return mcr.EvaluateFunction("TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd, thd, vsat);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut, Object positions)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut, Object positions, Object 
                                      destinations)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut, Object positions, Object 
                                      destinations, Object kH)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut, Object positions, Object 
                                      destinations, Object kH, Object kG)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut, Object positions, Object 
                                      destinations, Object kH, Object kG, Object ks)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut, Object positions, Object 
                                      destinations, Object kH, Object kG, Object ks, 
                                      Object kg)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks, kg);
    }


    /// <summary>
    /// Provides the standard 7-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut, Object positions, Object 
                                      destinations, Object kH, Object kG, Object ks, 
                                      Object kg, Object kth)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth);
    }


    /// <summary>
    /// Provides the standard 8-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut, Object positions, Object 
                                      destinations, Object kH, Object kG, Object ks, 
                                      Object kg, Object kth, Object sd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd);
    }


    /// <summary>
    /// Provides the standard 9-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut, Object positions, Object 
                                      destinations, Object kH, Object kG, Object ks, 
                                      Object kg, Object kth, Object sd, Object thd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd, thd);
    }


    /// <summary>
    /// Provides the standard 10-input Object interface to the TransportationControl3D
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
    public Object[] TransportationControl3D(int numArgsOut, Object positions, Object 
                                      destinations, Object kH, Object kG, Object ks, 
                                      Object kg, Object kth, Object sd, Object thd, 
                                      Object vsat)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D", positions, destinations, kH, kG, ks, kg, kth, sd, thd, vsat);
    }


    /// <summary>
    /// Provides an interface for the TransportationControl3D function in which the input
    /// and output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("TransportationControl3D", 10, 1, 0)]
    protected void TransportationControl3D(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("TransportationControl3D", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }
    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug()
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug(Object positions_in1)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug(Object positions_in1, Object 
                                          destinations_in1)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the
    /// TransportationControl3D_Debug MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="positions_in1">Input argument #1</param>
    /// <param name="destinations_in1">Input argument #2</param>
    /// <param name="kH">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object kH)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object kH, Object kG)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG);
    }


    /// <summary>
    /// Provides a single output, 5-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object kH, Object kG, Object 
                                          ks)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks);
    }


    /// <summary>
    /// Provides a single output, 6-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object kH, Object kG, Object 
                                          ks, Object kg)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg);
    }


    /// <summary>
    /// Provides a single output, 7-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object kH, Object kG, Object 
                                          ks, Object kg, Object kth)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth);
    }


    /// <summary>
    /// Provides a single output, 8-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object kH, Object kG, Object 
                                          ks, Object kg, Object kth, Object sd)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd);
    }


    /// <summary>
    /// Provides a single output, 9-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object kH, Object kG, Object 
                                          ks, Object kg, Object kth, Object sd, Object 
                                          thd)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd, thd);
    }


    /// <summary>
    /// Provides a single output, 10-input Objectinterface to the
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
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object TransportationControl3D_Debug(Object positions_in1, Object 
                                          destinations_in1, Object kH, Object kG, Object 
                                          ks, Object kg, Object kth, Object sd, Object 
                                          thd, Object vsat)
    {
      return mcr.EvaluateFunction("TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd, thd, vsat);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut, Object positions_in1)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object kH)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object kH, Object kG)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object kH, Object 
                                            kG, Object ks)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object kH, Object 
                                            kG, Object ks, Object kg)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg);
    }


    /// <summary>
    /// Provides the standard 7-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object kH, Object 
                                            kG, Object ks, Object kg, Object kth)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth);
    }


    /// <summary>
    /// Provides the standard 8-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object kH, Object 
                                            kG, Object ks, Object kg, Object kth, Object 
                                            sd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd);
    }


    /// <summary>
    /// Provides the standard 9-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object kH, Object 
                                            kG, Object ks, Object kg, Object kth, Object 
                                            sd, Object thd)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd, thd);
    }


    /// <summary>
    /// Provides the standard 10-input Object interface to the
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
    public Object[] TransportationControl3D_Debug(int numArgsOut, Object positions_in1, 
                                            Object destinations_in1, Object kH, Object 
                                            kG, Object ks, Object kg, Object kth, Object 
                                            sd, Object thd, Object vsat)
    {
      return mcr.EvaluateFunction(numArgsOut, "TransportationControl3D_Debug", positions_in1, destinations_in1, kH, kG, ks, kg, kth, sd, thd, vsat);
    }


    /// <summary>
    /// Provides an interface for the TransportationControl3D_Debug function in which the
    /// input and output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ++++++++++ Optional parameters evaluation ++++++++++
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("TransportationControl3D_Debug", 10, 15, 0)]
    protected void TransportationControl3D_Debug(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("TransportationControl3D_Debug", numArgsOut, ref argsOut, argsIn, varArgsIn);
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
