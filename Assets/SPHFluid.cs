using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using TMPro;
using UnityEngine.SceneManagement;

public class SPHSimulation : MonoBehaviour
{
    [Header("UI elements")]
    [SerializeField] private Button playButton;
    [SerializeField] private Button restartButton;
    
    [SerializeField] private Slider particleCountSlider;
    [SerializeField] private TextMeshProUGUI particleCountValue;
    [SerializeField] private TextMeshProUGUI particleCountLabel;

    [SerializeField] private Slider viscositySlider;
    [SerializeField] private TextMeshProUGUI viscosityValue;
    [SerializeField] private TextMeshProUGUI viscosityLabel;

    [SerializeField] private Slider gravitySlider;
    [SerializeField] private TextMeshProUGUI gravityValue;
    [SerializeField] private TextMeshProUGUI gravityLabel;

    [SerializeField] private TextMeshProUGUI startInstructions;
    [SerializeField] private TextMeshProUGUI playInstructions;
    private bool playing;

    [Header("Simulation Settings")]
    public int particleCount = 900;
    public float fluidMass = 2f;
    public float viscosityConst = 1f;
    public float dragStrength = 5f;
    public float gravity = 1.5f;
    public Vector3 border = new Vector3(1.8f, 2f, 0.8f);
    [Range (0, 1)]public float bounciness = 0.8f;
    public float particleSize = 0.05f;
    public bool constantTimeStep = false;
    public float timeStep = 0.1f;
    
    private float smoothingLength = 0.1f;
    private float smoothingRadius = 0.45f;
    private float normalizationConst = 0.1f;
    private float polytropicIndex = 1f;
    private float particleMass;
    private NativeArray<float3> positions;
    private NativeArray<float3> velocities;
    private NativeArray<float3> accelerations;
    private NativeArray<float> densities;
    private NativeArray<float> pressures;
    private GameObject[] particleObjects;
    private Camera mainCamera;
    
    [Header("Color Settings")]
    public Gradient gradient;
    public float maxSpeed = 1f;

    [Header("Interaction Settings")]
    public float interactionRadius = 0.5f;
    public float interactionStrength = 10f;
    private bool isMouseDragging = false;
    private Vector3 mouseWorldPosition;
    private int attraction = 1;

    void Start()
    {
        InitializeParticles();

        // Set gradient for particle colors  
        gradient = new Gradient();
        var colors = new GradientColorKey[4];
        colors[0] = new GradientColorKey(new Color(34f / 255f, 87f / 255f, 185f / 255f, 1f) , 0.06f);
        colors[1] = new GradientColorKey(new Color(76f / 255f, 1f, 144f / 255f, 1f), 0.5f);
        colors[2] = new GradientColorKey(new Color(1f, 237f / 255f, 0f, 1f), 0.71f);
        colors[3] = new GradientColorKey(new Color(247f / 255f, 73f / 255f, 8f / 255f, 1f), 1f);
        var alphas = new GradientAlphaKey[2];
        alphas[0] = new GradientAlphaKey(1.0f, 0.0f);
        alphas[1] = new GradientAlphaKey(1.0f, 1.0f);

        gradient.SetKeys(colors, alphas);

        // Set main camera for mouse interaction position calculation and assign particle mass
        mainCamera = Camera.main;

        // UI
        restartButton.gameObject.SetActive(false);
        viscosityLabel.gameObject.SetActive(false);
        viscositySlider.gameObject.SetActive(false);
        viscosityValue.gameObject.SetActive(false);
        gravityLabel.gameObject.SetActive(false);
        gravitySlider.gameObject.SetActive(false);
        gravityValue.gameObject.SetActive(false);
        playInstructions.gameObject.SetActive(false);

        playButton.onClick.AddListener(Play);
        restartButton.onClick.AddListener(Restart);

        particleCountSlider.value = particleCount;
        particleCountValue.text = particleCount.ToString();
        particleCountSlider.onValueChanged.AddListener(delegate { UpdateParticleCount(); });

        viscositySlider.value = viscosityConst;
        viscosityValue.text = viscosityConst.ToString("0.00");
        viscositySlider.onValueChanged.AddListener(delegate { UpdateViscosity(); });

        gravitySlider.value = gravity;
        gravityValue.text = gravity.ToString("0.00");
        gravitySlider.onValueChanged.AddListener(delegate { UpdateGravity(); });
        
        
    }

    void OnDestroy()
    {
        // Clear native arrays from memory
        if (positions.IsCreated) positions.Dispose();
        if (velocities.IsCreated) velocities.Dispose();
        if (accelerations.IsCreated) accelerations.Dispose();
        if (densities.IsCreated) densities.Dispose();
        if (pressures.IsCreated) pressures.Dispose();

        // Remove UI button listeners
        restartButton.onClick.RemoveListener(Restart);
        playButton.onClick.RemoveListener(Play);
        particleCountSlider.onValueChanged.RemoveListener(delegate { UpdateParticleCount(); });
        viscositySlider.onValueChanged.RemoveListener(delegate { UpdateViscosity(); });
        gravitySlider.onValueChanged.RemoveListener(delegate { UpdateGravity(); });
    }

    void Update()
    {
        // update simulation only if it is playing
        if (playing)
        {
            HandleMouseInput();
            // update particles with constant time step or the variable Time.deltaTime
            if (constantTimeStep)
            {
                UpdateParticles(timeStep);
            } else
            {
                UpdateParticles(Time.deltaTime);
            }
        }
    }


    void UpdateParticleCount()
    {
        float value = particleCountSlider.value;
        particleCountValue.text = value.ToString();
        particleCount = (int)value;
    }

    void UpdateViscosity()
    {
        float value = viscositySlider.value;
        viscosityConst = value;
        viscosityValue.text = value.ToString("0.00");
    }

    void UpdateGravity()
    {
        float value = gravitySlider.value;
        gravityValue.text = value.ToString("0.00");
        gravity = value;
    }

    // Play the simulation and change the ui accordingly
    void Play()
    {
        playing = true;

        restartButton.gameObject.SetActive(true);
        playButton.gameObject.SetActive(false);
        particleMass = fluidMass / particleCount;
        SpawnParticles();
        particleCountSlider.gameObject.SetActive(false);
        particleCountLabel.gameObject.SetActive(false);
        particleCountValue.gameObject.SetActive(false);

        viscosityLabel.gameObject.SetActive(true);
        viscositySlider.gameObject.SetActive(true);
        viscosityValue.gameObject.SetActive(true);
        gravityLabel.gameObject.SetActive(true);
        gravitySlider.gameObject.SetActive(true);
        gravityValue.gameObject.SetActive(true);

        startInstructions.gameObject.SetActive(false);
        playInstructions.gameObject.SetActive(true);
    }

    // Go back to the home screen without the simulation and update the ui accordingly
    void Restart()
    {
        playing = false;

        restartButton.gameObject.SetActive(false);
        // restart scene
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
        playButton.gameObject.SetActive(true);
        particleCountSlider.gameObject.SetActive(false);
        particleCountSlider.gameObject.SetActive(true);
        particleCountLabel.gameObject.SetActive(true);
        particleCountValue.gameObject.SetActive(true);

        viscosityLabel.gameObject.SetActive(false);
        viscositySlider.gameObject.SetActive(false);
        viscosityValue.gameObject.SetActive(false);
        gravityLabel.gameObject.SetActive(false);
        gravitySlider.gameObject.SetActive(false);
        gravityValue.gameObject.SetActive(false);

        startInstructions.gameObject.SetActive(true);
        playInstructions.gameObject.SetActive(false);
    }

    // If left mouse button is pressed, particles in interaction radius are attracted to the mouse and if the right button is pressed then the particles are repelled from the mouse
    void HandleMouseInput()
    {
        if (Input.GetMouseButtonDown(0))
        {
            isMouseDragging = true;
            attraction = 1;
        }

        if (Input.GetMouseButtonUp(0))
        {
            isMouseDragging = false;
        }

        if (Input.GetMouseButtonDown(1))
        {
            isMouseDragging = true;
            attraction = -1;
        }

        if (Input.GetMouseButtonUp(1))
        {
            isMouseDragging = false;
        }

        if (isMouseDragging)
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            Plane plane = new Plane(Vector3.forward, Vector3.zero);

            if (plane.Raycast(ray, out float distance))
            {
                mouseWorldPosition = ray.GetPoint(distance);
            }
        }
    }

    void InitializeParticles()
    {
        // Init native arrays
        positions = new NativeArray<float3>(particleCount, Allocator.Persistent);
        velocities = new NativeArray<float3>(particleCount, Allocator.Persistent);
        accelerations = new NativeArray<float3>(particleCount, Allocator.Persistent);
        densities = new NativeArray<float>(particleCount, Allocator.Persistent);
        pressures = new NativeArray<float>(particleCount, Allocator.Persistent);
        particleObjects = new GameObject[particleCount];
    }

    void SpawnParticles()
    {
        // Put particles in random positions within the border bounds and give each a velocity with a random direction
        for (int i = 0; i < particleCount; i++)
        {
            positions[i] = new float3(
                UnityEngine.Random.Range(-border.x / 2, border.x / 2),
                UnityEngine.Random.Range(-border.y / 2, border.y / 2),
                UnityEngine.Random.Range(-border.z / 2, border.z / 2)
            );
            velocities[i] = UnityEngine.Random.insideUnitSphere * 0.4f;

            particleObjects[i] = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            // Set position and size of the particle game object
            particleObjects[i].transform.localScale = Vector3.one * particleSize;
            particleObjects[i].transform.position = positions[i];
            // remove collider from each particle game object
            Destroy(particleObjects[i].GetComponent<Collider>());
        }
    }

    /// <summary>
    /// Calculates densities, pressures and accelerations for each particle using the DensityPressureJob and AccelerationJob and updates the particles accordingly
    /// </summary>
    /// <param name="dt">Time passed since the last update</param>
    void UpdateParticles(float dt)
    {
        // Compute densities and pressures 'in parallel'
        ComputeDensityPressureJob densityJob = new ComputeDensityPressureJob
        {
            positions = positions,
            densities = densities,
            pressures = pressures,
            particleMass = particleMass,
            smoothingLength = smoothingLength,
            normalizationConst = normalizationConst,
            polytropicIndex = polytropicIndex,
            smoothingRadius = smoothingRadius
        };
        JobHandle densityJobHandle = densityJob.Schedule(particleCount, 64);
        densityJobHandle.Complete();

        // Compute accelerations 'in parallel'
        ComputeAccelerationJob accJob = new ComputeAccelerationJob
        {
            positions = positions,
            velocities = velocities,
            densities = densities,
            pressures = pressures,
            accelerations = accelerations,
            particleMass = particleMass,
            smoothingLength = smoothingLength,
            viscosityConst = viscosityConst,
            gravity = gravity,
            smoothingRadius = smoothingRadius,
            mousePosition = mouseWorldPosition,
            isMouseDragging = isMouseDragging,
            interactionRadius = interactionRadius,
            interactionStrength = interactionStrength,
            attraction = attraction
        };
        JobHandle accJobHandle = accJob.Schedule(particleCount, 64);
        accJobHandle.Complete();

        for (int i = 0; i < particleCount; i++)
        {
            // update velocity and position based on acceleration
            velocities[i] += accelerations[i] * dt;
            positions[i] += velocities[i] * dt;

            // check if position is outside of bounds
            CheckCollisions(i);

            // assign the new position to the particle
            particleObjects[i].transform.position = positions[i];

            // update size of particle
            particleObjects[i].transform.localScale = Vector3.one * particleSize;

            // update color of particle based on its speed
            float colorValue = math.length(velocities[i]) / maxSpeed;
            Color color = gradient.Evaluate(colorValue);
            particleObjects[i].GetComponent<Renderer>().material.color = color;
        }
    }


    /// <summary>
    /// Checks if the position at positions[index] is outside of bounds and if so it changes the position back to the border and changes the velocity to the other direction
    /// </summary>
    /// <param name="index">Index of the position being checked</param>
    void CheckCollisions(int index)
    {
        // half of the border taking into account the size of the particles
        Vector3 halfBorder = border / 2 - Vector3.one * particleSize;


        if (Mathf.Abs(positions[index].x) > halfBorder.x)
        {
            float newX = halfBorder.x * Mathf.Sign(positions[index].x);
            positions[index] = new float3(newX, positions[index].y, positions[index].z);
            float newXVel = velocities[index].x * (-bounciness);
            velocities[index] = new float3(newXVel, velocities[index].y, velocities[index].z);
        }
        if (Mathf.Abs(positions[index].y) > halfBorder.y)
        {
            float newY = halfBorder.y * Mathf.Sign(positions[index].y);
            positions[index] = new float3( positions[index].x, newY, positions[index].z);
            float newYVel = velocities[index].y * (-bounciness);
            velocities[index] = new float3(velocities[index].x, newYVel, velocities[index].z);
        }
        if (Mathf.Abs(positions[index].z) > halfBorder.z)
        {
            float newZ = halfBorder.z * Mathf.Sign(positions[index].z);
            positions[index] = new float3(positions[index].x, positions[index].y, newZ);
            float newZVel = velocities[index].z * (-bounciness);
            velocities[index] = new float3(velocities[index].x, velocities[index].y, newZVel);
        }
    }


    // Computes density and pressure based on eulers equations using the Gaussian Smoothing Function as an approximation to the Dirac Delta Function
    [BurstCompile]
    private struct ComputeDensityPressureJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> positions;
        public NativeArray<float> densities;
        public NativeArray<float> pressures;
        public float particleMass, smoothingLength, normalizationConst, polytropicIndex, smoothingRadius;

        public void Execute(int i)
        {
            float density = 0f;
            for (int j = 0; j < positions.Length; j++)
            {
                float3 difference = positions[i] - positions[j];
                float distance = math.length(difference);
                if (distance < smoothingRadius)
                {
                    density += particleMass * GaussianSmoothingKernel(distance, smoothingLength);
                }
            }
            densities[i] = density;
            // Equation of state
            pressures[i] = normalizationConst * math.pow(density, 1 + 1 / polytropicIndex);
        }

        private float GaussianSmoothingKernel(float r, float h)
        {
            return math.pow(1.0f / (h * math.sqrt(math.PI)), 3) * math.exp(-r * r / (h * h));
        }
    }


    // Computes acceleration based on eulers equations using the Gaussian Smoothing Function Derivative
    [BurstCompile]
    private struct ComputeAccelerationJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> positions;
        [ReadOnly] public NativeArray<float3> velocities;
        [ReadOnly] public NativeArray<float> densities;
        [ReadOnly] public NativeArray<float> pressures;
        public NativeArray<float3> accelerations;
        public float particleMass, smoothingLength, viscosityConst, gravity, smoothingRadius;
        // Interaction variables
        [ReadOnly] public float3 mousePosition;
        [ReadOnly] public bool isMouseDragging;
        public float interactionRadius;
        public float interactionStrength;
        public int attraction;

        public void Execute(int i)
        {
            float3 acc = new float3(0, -gravity, 0);
            for (int j = 0; j < positions.Length; j++)
            {
                if (i != j)
                {
                    float3 difference = positions[i] - positions[j];
                    float distance = math.length(difference);
                    if (distance < smoothingRadius && distance > 0)
                    {
                        float3 gradW = GaussianSmoothingKernelDerivative(difference, distance, smoothingLength);
                        float factor = particleMass * (pressures[i] / (densities[i] * densities[i]) + pressures[j] / (densities[j] * densities[j]));
                        acc -= factor * gradW;
                    }
                }
            }

            // Mouse interaction
            if (isMouseDragging)
            {
                // If particle is withing the radius of the mouse position add the corresponding force to its acceleration
                float3 toMouse = mousePosition - positions[i];
                float distance = math.length(toMouse);
                if (distance < interactionRadius)
                {
                    float interactionFactor = interactionStrength * (1 - distance / interactionRadius);
                    acc += math.normalize(toMouse) * interactionFactor * attraction;
                }
            }

            // Add viscosity to acceleration
            acc -= viscosityConst * velocities[i];
            accelerations[i] = acc;
        }

        private float3 GaussianSmoothingKernelDerivative(float3 difference, float distance, float smoothingLength)
        {
            float factor = -2 * math.exp(-distance * distance / (smoothingLength * smoothingLength)) / math.pow(smoothingLength, 5) / math.pow(math.PI, 3.0f / 2.0f);
            return factor * difference;
        }
    }
}

















