use crate::{Celsius, Milliseconds};

/// Einstellungen für das Oversampling von Temperatur, Druck und Feuchtigkeit.
///
/// Höhere Oversampling-Raten reduzieren das Rauschen durch Mittelwertbildung in der Hardware,
/// verlängern jedoch die Messdauer und erhöhen den Stromverbrauch pro Messzyklus.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(u8)]
pub enum Oversampling {
    /// Keine Messung. Deaktiviert den entsprechenden Sensorkanal vollständig.
    Skipped = 0,
    /// 1-faches Oversampling (Standard).
    #[default]
    X1 = 1,
    /// 2-faches Oversampling.
    X2 = 2,
    /// 4-faches Oversampling.
    X4 = 3,
    /// 8-faches Oversampling.
    X8 = 4,
    /// 16-faches Oversampling. Maximale Präzision, längste Dauer.
    X16 = 5,
}

impl Oversampling {
    /// Erzeugt eine Instanz aus einem Rohwert (hilfreich beim Parsen von Registern).
    pub fn from_u8(value: u8) -> Self {
        match value {
            1 => Oversampling::X1,
            2 => Oversampling::X2,
            3 => Oversampling::X4,
            4 => Oversampling::X8,
            5 => Oversampling::X16,
            _ => Oversampling::Skipped,
        }
    }
}

/// Gruppierte Oversampling-Konfiguration für alle Umwelt-Sensoren.
///
/// Nutzen Sie `Oversampling::Skipped`, um Kanäle zu deaktivieren, die für Ihre
/// Anwendung nicht relevant sind (spart Zeit und Energie).
#[derive(Default, Debug, Clone, Copy)]
pub struct OversamplingConfig {
    /// Oversampling für den Temperatursensor.
    pub temp_osrs: Oversampling,
    /// Oversampling für den Luftfeuchtigkeitssensor.
    pub hum_osrs: Oversampling,
    /// Oversampling für den Luftdrucksensor.
    pub pres_osrs: Oversampling,
}

impl OversamplingConfig {
    /// Gibt `true` zurück, wenn alle TPH-Sensoren auf `Skipped` stehen.
    pub fn is_all_skipped(&self) -> bool {
        self.temp_osrs == Oversampling::Skipped
            && self.hum_osrs == Oversampling::Skipped
            && self.pres_osrs == Oversampling::Skipped
    }
}

/// Koeffizient für den IIR (Infinite Impulse Response) Filter.
///
/// Der Filter glättet kurzfristige Störungen in den Druck- und Temperaturwerten
/// (z.B. durch zuschlagende Türen oder Luftzüge). Er hat keinen Einfluss auf
/// Feuchtigkeit oder Gas.
#[derive(Default, Debug, Clone, Copy)]
#[repr(u8)]
pub enum IIRFilter {
    /// Filter deaktiviert.
    #[default]
    IIR0 = 0,
    IIR1 = 1,
    IIR3 = 2,
    IIR7 = 3,
    IIR15 = 4,
    IIR31 = 5,
    IIR63 = 6,
    IIR127 = 7,
}

/// Verfügbare Speicherplätze (Slots) für Heizprofile im Sensor (0 bis 9).
#[derive(Default, Debug, Clone, Copy)]
#[repr(u8)]
pub enum GasProfileIndex {
    #[default]
    Profile0 = 0,
    Profile1 = 1,
    Profile2 = 2,
    Profile3 = 3,
    Profile4 = 4,
    Profile5 = 5,
    Profile6 = 6,
    Profile7 = 7,
    Profile8 = 8,
    Profile9 = 9,
}

/// Konfiguration für die Heizplatte des Gassensors.
#[derive(Default, Debug, Clone, Copy)]
pub struct GasProfile {
    /// Index des Profil-Slots im Sensor-Speicher.
    pub index: GasProfileIndex,
    /// Zieltemperatur der Heizplatte (typischerweise 300°C bis 400°C).
    pub target_temp: Celsius,
    /// Dauer, für die die Temperatur vor der Messung gehalten wird (Aufheizzeit).
    pub wait_time: Milliseconds,
}

/// Vollständiges Konfigurationsobjekt für den BME680.
#[derive(Default, Debug, Clone, Copy)]
pub struct Config {
    /// Oversampling-Einstellungen für T, P und H.
    pub osrs_config: OversamplingConfig,
    /// IIR-Filter-Einstellung zur Rauschunterdrückung.
    pub iir_filter: IIRFilter,
    /// Gassensor-Profil. `None` deaktiviert die Heizung und die Gasmessung (spart ~12mA).
    pub gas_profile: Option<GasProfile>,
    /// Aktuelle Schätzung der Umgebungstemperatur.
    /// Wichtig für die korrekte Berechnung der Heizleistung.
    pub ambient_temp: Celsius,
}

impl Config {
    /// Prüft, ob die Gasmessung laut Konfiguration aktiv ist.
    pub(crate) fn gas_enabled(&self) -> bool {
        self.gas_profile.is_some()
    }

    /// Prüft, ob die Gasmessung deaktiviert ist.
    pub(crate) fn gas_disabled(&self) -> bool {
        self.gas_profile.is_none()
    }
}

/// Komfortabler Builder zum Erstellen einer `Config`.
///
/// Der Builder stellt sicher, dass alle Parameter logisch zusammenhängen und
/// bietet eine saubere API für die Initialisierung.
#[derive(Default)]
pub struct BME680Builder {
    config: Config,
}

impl BME680Builder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Setzt das Oversampling für die Temperatur.
    pub fn temp_oversampling(mut self, os: Oversampling) -> Self {
        self.config.osrs_config.temp_osrs = os;
        self
    }

    /// Setzt das Oversampling für die Luftfeuchtigkeit.
    pub fn hum_oversampling(mut self, os: Oversampling) -> Self {
        self.config.osrs_config.hum_osrs = os;
        self
    }

    /// Setzt das Oversampling für den Luftdruck.
    pub fn pres_oversampling(mut self, os: Oversampling) -> Self {
        self.config.osrs_config.pres_osrs = os;
        self
    }

    /// Setzt den IIR-Filter-Koeffizienten.
    pub fn iir_filter(mut self, filter: IIRFilter) -> Self {
        self.config.iir_filter = filter;
        self
    }

    /// Aktiviert oder deaktiviert das Gasprofil.
    pub fn gas_profile(mut self, profile: Option<GasProfile>) -> Self {
        self.config.gas_profile = profile;
        self
    }

    /// Setzt die initial geschätzte Umgebungstemperatur für die Heizungsberechnung.
    pub fn ambient_temp(mut self, temp: Celsius) -> Self {
        self.config.ambient_temp = temp;
        self
    }

    /// Finalisiert den Builder und gibt das `Config` Objekt zurück.
    pub fn build(self) -> Config {
        self.config
    }
}
