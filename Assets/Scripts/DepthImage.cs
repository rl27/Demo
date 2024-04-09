using System;
using System.Text;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;

// ARFoundation references:
// https://github.com/Unity-Technologies/arfoundation-samples/blob/main/Assets/Scripts/Runtime/DisplayDepthImage.cs
// https://github.com/Unity-Technologies/arfoundation-samples/blob/main/Assets/Scripts/Runtime/CpuImageSample.cs
public class DepthImage : MonoBehaviour
{
    public Text imageInfo {
        get => m_ImageInfo;
        set => m_ImageInfo = value;
    }
    [SerializeField]
    Text m_ImageInfo;

    public Text obstacleInfo {
        get => m_ObstacleInfo;
        set => m_ObstacleInfo = value;
    }
    [SerializeField]
    Text m_ObstacleInfo;

    // Get or set the AROcclusionManager.
    public AROcclusionManager occlusionManager {
        get => m_OcclusionManager;
        set => m_OcclusionManager = value;
    }
    [SerializeField]
    [Tooltip("The AROcclusionManager which will produce depth textures.")]
    AROcclusionManager m_OcclusionManager;

    // Get or set the ARCameraManager.
    public ARCameraManager cameraManager {
        get => m_CameraManager;
        set => m_CameraManager = value;
    }
    [SerializeField]
    [Tooltip("The ARCameraManager which will produce camera frame events.")]
    ARCameraManager m_CameraManager;

    // Using multiple audio sources to queue collision audio with no delay
    // https://johnleonardfrench.com/ultimate-guide-to-playscheduled-in-unity/#queue_clips
    // https://docs.unity3d.com/ScriptReference/AudioSource.SetScheduledEndTime.html
    public AudioSource[] audioSources;
    private int audioSelect = 0; // Select audio source
    private double lastScheduled = -10; // mag = -1 for left, mag = 1 for right

    public static float collisionAudioMinRate = 2f; // Rate at which audio plays for obstacles at max distance
    public static float collisionAudioCapDistance = 0.5f; // Distance where audio speed caps out
    private double audioDuration; // Collision audio duration = 0.0853333333333333 (if using audioclip.length, it's 0.08533333)
    private float collisionAudioMaxRate; // 11.72f

    // The UI RawImage used to display the image on screen.
    public RawImage rawImage {
        get => m_RawImage;
        set => m_RawImage = value;
    }
    [SerializeField]
    RawImage m_RawImage;

    // This is for using a custom shader that lets us see the full range of depth.
    // See the Details section here: https://github.com/andijakl/arfoundation-depth
    public Material depthMaterial {
        get => m_DepthMaterial;
        set => m_DepthMaterial = value;
    }
    [SerializeField]
    Material m_DepthMaterial;

    // Depth image data
    byte[] depthArray = new byte[0];
    int depthWidth = 0; // (width, height) = (160, 90) on OnePlus 11; (256, 192) on iPhone 12 Pro
    int depthHeight = 0;
    int depthStride = 4; // Should be either 2 or 4
    
    // Camera intrinsics
    Vector2 focalLength = Vector2.zero;
    Vector2 principalPoint = Vector2.zero;

    // Converts local coordinates to world coordinates.
    private Matrix4x4 localToWorldTransform = Matrix4x4.identity;
    private Matrix4x4 screenRotation = Matrix4x4.Rotate(Quaternion.identity);
    private new Camera camera;

    public static Vector3 position;
    public static Vector3 rotation;

    // These variables are for obstacle avoidance.
    private bool doObstacleAvoidance = false;
    public static float distanceToObstacle = 2.5f; // Distance in meters at which to alert for obstacles

    public Slider slider;
    public Text sliderText;

    public Toggle depthToggle;

    double curTime = 0;
    double lastDSP = 0;

    void Awake()
    {
        Application.targetFrameRate = 60;

        camera = m_CameraManager.GetComponent<Camera>();

        m_CameraManager.frameReceived += OnCameraFrameReceived;

        // Set depth image material
        m_RawImage.material = m_DepthMaterial;

        m_RawImage.enabled = doObstacleAvoidance;

        audioDuration = (double) audioSources[0].clip.samples / audioSources[0].clip.frequency;
        collisionAudioMaxRate = 1 / (float)audioDuration;

        slider.value = distanceToObstacle;
        sliderText.text = distanceToObstacle.ToString("F2") + "m";
        slider.onValueChanged.AddListener((val) => {distanceToObstacle = val; sliderText.text = val.ToString("F2") + "m";});
    }

    void OnCameraFrameReceived(ARCameraFrameEventArgs args)
    {
        // if (doObstacleAvoidance)
            UpdateDepthImages();
    }

    // This is called every frame
    void Update()
    {
        // Update timer for collision audio
        if (lastDSP != AudioSettings.dspTime) {
            lastDSP = AudioSettings.dspTime;
            curTime = lastDSP;
        }
        else curTime += Time.unscaledDeltaTime;

        // position and rotation briefly become 0 on focus loss/regain, which can mess things up
        // Same for localToWorldTransform
        if (camera.transform.position != Vector3.zero && camera.transform.rotation.eulerAngles != Vector3.zero) {
            position = camera.transform.position;
            rotation = camera.transform.rotation.eulerAngles;
        }
        screenRotation = Matrix4x4.Rotate(Quaternion.Euler(0, 0, GetRotationForScreen()));
        if (camera.transform.localToWorldMatrix != Matrix4x4.identity)
            localToWorldTransform = camera.transform.localToWorldMatrix * screenRotation;

        // Check for obstacle
            float center = GetDepth(depthWidth/2-1, depthHeight/2-1);
            if (center < 9999)
                m_ImageInfo.text = String.Format("{0}m", center.ToString("F2"));
            if (center < distanceToObstacle) {
                m_ObstacleInfo.text = "Obstacle ahead!";
                float rate = (center - collisionAudioCapDistance) / (distanceToObstacle - collisionAudioCapDistance);
                rate = Mathf.Lerp(collisionAudioMaxRate, collisionAudioMinRate, rate);

                PlayCollision(0, 1/rate - audioDuration);
            }
            else {
                m_ObstacleInfo.text = "";
            }
    }

    private bool UpdateDepthImages()
    {
        bool success = false;

        // Acquire a depth image and update the corresponding raw image.
        if (m_OcclusionManager.TryAcquireEnvironmentDepthCpuImage(out XRCpuImage image)) {
            using (image) {
                UpdateRawImage(m_RawImage, image, image.format.AsTextureFormat());

                // Get distance data into depthArray
                // https://github.com/googlesamples/arcore-depth-lab/blob/8f76532d4a67311463ecad6b88b3f815c6cf1eea/Assets/ARRealismDemos/Common/Scripts/MotionStereoDepthDataSource.cs#L250
                depthWidth = image.width;
                depthHeight = image.height;
                UpdateCameraParams();

                int numPixels = depthWidth * depthHeight;
                Debug.Assert(image.planeCount == 1, "Plane count is not 1");
                depthStride = image.GetPlane(0).pixelStride;
                int numBytes = numPixels * depthStride;
                if (depthArray.Length != numBytes)
                    depthArray = new byte[numBytes];
                image.GetPlane(0).data.CopyTo(depthArray);

                success = true;
            }
        }

        return success;
    }

    // Need to handle multiple audio sources so we can schedule sufficiently ahead of time, particularly at low FPS.
    private void PlayCollision(float dir, double delay)
    {
        float rad = (dir + rotation.y) * Mathf.Deg2Rad;
        this.transform.position = position + new Vector3(Mathf.Sin(rad), 0, Mathf.Cos(rad));

        double nextSchedule = Math.Max(curTime, lastScheduled + audioDuration + delay);
        while (nextSchedule - curTime < 0.2 && !audioSources[audioSelect].isPlaying) { // Schedule next audio if it will be needed soon
            audioSources[audioSelect].PlayScheduled(nextSchedule);
            audioSelect = (audioSelect + 1) % audioSources.Length;
            lastScheduled = nextSchedule;
            nextSchedule = Math.Max(curTime, lastScheduled + audioDuration + delay);
        }
    }

    public void ToggleObstacleAvoidance()
    {
        doObstacleAvoidance = depthToggle.isOn;
        m_RawImage.enabled = doObstacleAvoidance;
        // m_OcclusionManager.enabled = doObstacleAvoidance;
        // m_ImageInfo.text = "";
        // m_ObstacleInfo.text = "";
    }

    private void UpdateRawImage(RawImage rawImage, XRCpuImage cpuImage, TextureFormat format)
    {
        Debug.Assert(rawImage != null, "no raw image");

        // Get the texture associated with the UI.RawImage that we wish to display on screen.
        var texture = rawImage.texture as Texture2D;

        // If the texture hasn't yet been created, or if its dimensions have changed, (re)create the texture.
        // Note: Although texture dimensions do not normally change frame-to-frame, they can change in response to
        //    a change in the camera resolution (for camera images) or changes to the quality of the human depth
        //    and human stencil buffers.
        if (texture == null || texture.width != cpuImage.width || texture.height != cpuImage.height)
        {
            texture = new Texture2D(cpuImage.width, cpuImage.height, format, false);
            rawImage.texture = texture;
        }

        // For display, we need to mirror about the vertical axis.
        var conversionParams = new XRCpuImage.ConversionParams(cpuImage, format, XRCpuImage.Transformation.MirrorY);

        // Get the Texture2D's underlying pixel buffer.
        var rawTextureData = texture.GetRawTextureData<byte>();

        // Make sure the destination buffer is large enough to hold the converted data (they should be the same size)
        Debug.Assert(rawTextureData.Length == cpuImage.GetConvertedDataSize(conversionParams.outputDimensions, conversionParams.outputFormat),
            "The Texture2D is not the same size as the converted data.");

        // Perform the conversion.
        cpuImage.Convert(conversionParams, rawTextureData);

        // "Apply" the new pixel data to the Texture2D.
        texture.Apply();

        Vector2 rectSize;
        switch (Screen.orientation)
        {
            case ScreenOrientation.LandscapeRight:
            case ScreenOrientation.LandscapeLeft:
                rectSize = new Vector2(Screen.width, (float) Screen.width * texture.height / texture.width);
                break;
            case ScreenOrientation.PortraitUpsideDown:
            case ScreenOrientation.Portrait:
            default:
                rectSize = new Vector2((float) Screen.height * texture.height / texture.width, Screen.height);
                break;
        }
        rawImage.rectTransform.sizeDelta = rectSize;

        // Rotate the depth material to match screen orientation.
        Quaternion rotation = Quaternion.Euler(0, 0, GetRotation());
        Matrix4x4 rotMatrix = Matrix4x4.Rotate(rotation);
        m_RawImage.material.SetMatrix(Shader.PropertyToID("_DisplayRotationPerFrame"), rotMatrix);
    }

    /*
    Obtain the depth value in meters. (u,v) are normalized screen coordinates; stride is the pixel stride of the acquired environment depth image.
    This function is based on: https://github.com/googlesamples/arcore-depth-lab/blob/8f76532d4a67311463ecad6b88b3f815c6cf1eea/Assets/ARRealismDemos/OrientedReticle/Scripts/OrientedReticle.cs#L116
    Further references:
    https://developers.google.com/ar/develop/unity-arf/depth/developer-guide#extract_distance_from_a_depth_image
    https://github.com/googlesamples/arcore-depth-lab/blob/8f76532d4a67311463ecad6b88b3f815c6cf1eea/Assets/ARRealismDemos/Common/Scripts/DepthSource.cs#L436
    */
    public float GetDepth(Vector2 uv)
    {
        if (depthArray.Length == 0)
            return 99999f;
        
        int x = (int)(uv.x * (depthWidth - 1));
        int y = (int)(uv.y * (depthHeight - 1));

        return GetDepth(x, y);
    }

    public float GetDepth(int x, int y)
    {
        if (depthArray.Length == 0)
            return 99999f;

        // if (x < 0 || x >= depthWidth || y < 0 || y >= depthHeight) {
        //     Debug.Log("Invalid depth index");
        //     return -99999f;
        // }

        /*
        On an iPhone 12 Pro, the image data is in DepthFloat32 format, so we use ToSingle().
        On a OnePlus 11, it is in DepthUInt16 format.
        CPU image formats are described here:
        https://docs.unity3d.com/Packages/com.unity.xr.arsubsystems@4.1/api/UnityEngine.XR.ARSubsystems.XRCpuImage.Format.html
        Also see the below example code:
        https://forum.unity.com/threads/how-to-measure-distance-from-depth-map.1440799
        */
        int index = (y * depthWidth) + x;
        float depthInMeters = 0;
        if (depthStride == 4) // DepthFloat32
            depthInMeters = BitConverter.ToSingle(depthArray, depthStride * index);
        else if (depthStride == 2) // DepthUInt16
            depthInMeters = BitConverter.ToUInt16(depthArray, depthStride * index) * 0.001f;

        if (depthInMeters > 0) {
            // Do not factor in focalLength and principalPoint if only measuring forward distance from camera
            /*float vertex_x = (x - principalPoint.x) * depthInMeters / focalLength.x;
            float vertex_y = (y - principalPoint.y) * depthInMeters / focalLength.y;
            return Mathf.Sqrt(vertex_x*vertex_x + vertex_y*vertex_y + depthInMeters*depthInMeters);*/
            return depthInMeters;
        }

        return 99999f;
    }

    // Given image pixel coordinates (x,y) and distance z, returns a vertex in local camera space.
    public Vector3 ComputeVertex(int x, int y, float z)
    {
        Vector3 vertex = Vector3.negativeInfinity;
        if (z > 0) {
            float vertex_x = (x - principalPoint.x) * z / focalLength.x;
            float vertex_y = (y - principalPoint.y) * z / focalLength.y;
            vertex.x = vertex_x;
            vertex.y = -vertex_y;
            vertex.z = z;
        }
        return vertex;
    }

    // Transforms a vertex in local space to world space
    // NOTE: Is not the same as using camera.ScreenToWorldPoint.
    // https://forum.unity.com/threads/how-to-get-point-cloud-in-arkit.967681/#post-6340404
    public Vector3 TransformLocalToWorld(Vector3 vertex)
    {
        return localToWorldTransform.MultiplyPoint(vertex);
    }

    // https://github.com/Unity-Technologies/arfoundation-samples/issues/266#issuecomment-523316133
    public static int GetRotation() => Screen.orientation switch
    {
        ScreenOrientation.Portrait => 90,
        ScreenOrientation.LandscapeLeft => 180,
        ScreenOrientation.PortraitUpsideDown => -90,
        ScreenOrientation.LandscapeRight => 0,
        _ => 90
    };

    private int GetRotationForScreen() => Screen.orientation switch
    {
        ScreenOrientation.Portrait => -90,
        ScreenOrientation.LandscapeLeft => 0,
        ScreenOrientation.PortraitUpsideDown => 90,
        ScreenOrientation.LandscapeRight => 180,
        _ => -90
    };

    // https://github.com/googlesamples/arcore-depth-lab/blob/8f76532d4a67311463ecad6b88b3f815c6cf1eea/Assets/ARRealismDemos/Common/Scripts/MotionStereoDepthDataSource.cs#L219
    private void UpdateCameraParams()
    {
        // Gets the camera parameters to create the required number of vertices.
        if (m_CameraManager.TryGetIntrinsics(out XRCameraIntrinsics cameraIntrinsics))
        {
            // Scales camera intrinsics to the depth map size.
            Vector2 intrinsicsScale;
            intrinsicsScale.x = depthWidth / (float)cameraIntrinsics.resolution.x;
            intrinsicsScale.y = depthHeight / (float)cameraIntrinsics.resolution.y;

            // intrinsicsScale: 0.25, 0.19
            // cameraIntrinsics.resolution: 640, 480
            // cameraIntrinsics.focalLength): 442.88, 443.52
            // cameraIntrinsics.principalPoint: 321.24, 239.47

            focalLength = MultiplyVector2(cameraIntrinsics.focalLength, intrinsicsScale); // On OnePlus 11: close to (110.32, 82.62), but should probably be (110,110)
            principalPoint = MultiplyVector2(cameraIntrinsics.principalPoint, intrinsicsScale); // This is always close to (depthWidth/2, depthHeight/2)

            // focalLength.y is not accurate
            // Inspired by: https://github.com/googlesamples/arcore-depth-lab/blob/8f76532d4a67311463ecad6b88b3f815c6cf1eea/Assets/ARRealismDemos/OrientedReticle/Scripts/OrientedReticle.cs#L240
            focalLength.y = focalLength.x;
        }
    }

    private Vector2 MultiplyVector2(Vector2 v1, Vector2 v2)
    {
        return new Vector2(v1.x * v2.x, v1.y * v2.y);
    }
}