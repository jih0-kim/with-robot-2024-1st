using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class DepthCameraPublisher : MonoBehaviour
{
    [SerializeField] private string topicName = "jetauto/detp_camera/img_raw";
    [SerializeField] private float publishFrequency = 0.1f;
    [SerializeField] private RenderTexture renderTexture = null;

    ROSConnection ros;
    private float timeElapsed = 0.0f;


    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
    }

    private void FixedUpdate()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishFrequency)
        {
            Texture2D texture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGBA32, false);
            RenderTexture.active = renderTexture;
            texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0, false);
     
            ImageMsg img_raw = texture.ToImageMsg(new HeaderMsg());
            ros.Publish(topicName, img_raw);

            timeElapsed = 0;
        }
    }
}
